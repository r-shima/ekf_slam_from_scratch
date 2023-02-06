/// \file
/// \brief This node displays a red turtlebot in a simulated environment with cylindrical obstacles
///
/// PARAMETERS:
///     rate (int): a frequency used to run the main loop
///     x0 (double): the x component of the initial pose of the robot
///     y0 (double): the y component of the initial pose of the robot
///     theta0 (double): the theta component of the initial pose of the robot
///     obstacles.x (std::vector<double>): a list of the obstacles' x coordinates
///     obstacles.y (std::vector<double>): a list of the obstacles' y coordinates
///     obstacles.r (double): the radius of the obstacles
/// PUBLISHES:
///     /nusim/timestep (std_msgs::msg::UInt64): the timestep for the simulation
///     /nusim/obstacles (visualization_msgs::msg::MarkerArray): cylinderical markers that act as
///                                                              the obstacles
/// SERVERS:
///     reset (std_srvs::srv::Empty): restores the initial state of the simulation and the robot's
///                                   initial location
///     teleport (nusim::srv::Teleport): enables moving the robot to a desired (x, y, theta) pose

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

/// \brief Creates a simulation environment for the turtlebot
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"),
    timestep_(0)
  {
    // Initializes variables for the parameters, publishers, timer, services, and broadcaster
    declare_parameter("rate", 200);
    declare_parameter("x0", 0.0);
    declare_parameter("y0", 0.0);
    declare_parameter("theta0", 0.0);
    declare_parameter("obstacles.x", std::vector<double> {});
    declare_parameter("obstacles.y", std::vector<double> {});
    declare_parameter("obstacles.r", 0.038);
    declare_parameter("walls.x_length", 5.0);
    declare_parameter("walls.y_length", 5.0);
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
    declare_parameter("collision_radius", -1.0);
    rate_ = get_parameter("rate").get_parameter_value().get<int>();
    x0_ = get_parameter("x0").get_parameter_value().get<double>();
    y0_ = get_parameter("y0").get_parameter_value().get<double>();
    theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
    obstacles_x_ = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obstacles_y_ = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();
    walls_x_length_ = get_parameter("walls.x_length").get_parameter_value().get<double>();
    walls_y_length_ = get_parameter("walls.y_length").get_parameter_value().get<double>();
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    wall_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this,
      std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&Nusim::timer_callback, this));
    reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(
        &Nusim::reset_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    teleport_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(
        &Nusim::teleport_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;

    thickness_ = 0.15;
    position_x_ = {walls_x_length_ / 2 + thickness_ / 2, -walls_x_length_ / 2 - thickness_ / 2,
                   0.0, 0.0};
    position_y_ = {0.0, 0.0, walls_y_length_ / 2 + thickness_ / 2,
                   -walls_y_length_ / 2 - thickness_ / 2};
    scale_x_ = {thickness_, thickness_, walls_x_length_ + 2.0 * thickness_,
                walls_x_length_ + 2.0 * thickness_};
    scale_y_ = {walls_y_length_, walls_y_length_, thickness_, thickness_};

    check_params();
    add_obstacles();
    add_walls();
  }

private:
  /// \brief Callback function for the timer. Broadcasts the transform and publishes a marker array.
  ///
  /// \param none
  /// \returns none
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;
    RCLCPP_INFO_STREAM(get_logger(), "Publishing: " << message.data);
    timestep_pub_->publish(message);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
    marker_pub_->publish(marker_array_);
    wall_marker_pub_->publish(wall_array_);
    sensor_data_pub_->publish(sensor_data_);
  }

  /// \brief Callback function for the reset service. Resets the timestep and restores the initial
  ///        pose of the robot.
  ///
  /// \param request - not being used
  /// \param response - not being used
  /// \returns none
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    timestep_ = 0;
    x_ = x0_;
    y_ = y0_;
    theta_ = theta0_;
  }

  /// \brief Callback function for the teleport service. Teleports the robot to a desired pose.
  ///
  /// \param request - x, y, and theta components of the desired pose
  /// \param response - not being used
  /// \returns none
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_ = request->x;
    y_ = request->y;
    theta_ = request->theta;
  }

  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg) {
    sensor_data_.stamp = this->get_clock()->now();
    sensor_data_.left_encoder = msg.left_velocity;
    sensor_data_.right_encoder = msg.right_velocity;
    angle_.l = msg.left_velocity / encoder_ticks_per_rad_;
    angle_.r = msg.right_velocity / encoder_ticks_per_rad_;
    diff_drive_.forward_kinematics(angle_);
    x_ = diff_drive_.configuration().x;
    y_ = diff_drive_.configuration().y;
    theta_ = diff_drive_.configuration().theta;
    // x0_ = x_;
    // y0_ = y_;
    // theta0_ = theta_;
  }

  void check_params() {
    if(wheel_radius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1 ||
       motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1.0 ||
       collision_radius_ == -1.0) {
        int num = 0;
        RCLCPP_ERROR(this->get_logger(), "The parameters are not defined");
        throw num;
    }
  }

  /// \brief Creates markers for the cylindrical obstacles and adds them to a marker array
  ///
  /// \param none
  /// \returns none
  void add_obstacles() {
    const auto marker_array_size = obstacles_x_.size();

    if (obstacles_x_.size() != obstacles_y_.size()) {
      throw(std::runtime_error("Obstacles' x and y coordinate lists do not have the same length"));
    }

    for (size_t i = 0; i < marker_array_size; i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = this->get_clock()->now();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_x_.at(i);
      marker.pose.position.y = obstacles_y_.at(i);
      marker.pose.position.z = 0.125;
      marker.scale.x = 2.0 * obstacles_r_;
      marker.scale.y = 2.0 * obstacles_r_;
      marker.scale.z = 0.25;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      marker_array_.markers.push_back(marker);
    }
  }

  void add_walls()
  {
    for (int i = 0; i < 4; i++) {
        visualization_msgs::msg::Marker wall_marker;
        wall_marker.header.frame_id = "nusim/world";
        wall_marker.header.stamp = this->get_clock()->now();
        wall_marker.id = i;
        wall_marker.type = visualization_msgs::msg::Marker::CUBE;
        wall_marker.action = visualization_msgs::msg::Marker::ADD;
        wall_marker.pose.position.x = position_x_.at(i);
        wall_marker.pose.position.y = position_y_.at(i);
        wall_marker.scale.x = scale_x_.at(i);
        wall_marker.scale.y = scale_y_.at(i);
        wall_marker.color.r = 0.3;
        wall_marker.color.g = 0.5;
        wall_marker.color.b = 1.0;
        wall_marker.color.a = 1.0;
        wall_marker.pose.position.z = 0.125;
        wall_marker.scale.z = 0.25;
        wall_array_.markers.push_back(wall_marker);
    }
  }

  // Declare private variables for the publishers, a timer, services, and a broadcaster
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_marker_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Declare private variables for the node parameters, a timestep, a marker array, and the x, y,
  // and theta components of the robot's initial location
  size_t timestep_;
  int rate_;
  double x0_, y0_, theta0_; // x_init_, y_init_, theta_init_
  std::vector<double> obstacles_x_, obstacles_y_;
  double obstacles_r_, walls_x_length_, walls_y_length_;
  visualization_msgs::msg::MarkerArray marker_array_;
  std::vector<double> position_x_, position_y_, scale_x_, scale_y_;
  visualization_msgs::msg::MarkerArray wall_array_;
  double thickness_;
  turtlelib::WheelAngle angle_;
  double wheel_radius_, track_width_, motor_cmd_per_rad_sec_, encoder_ticks_per_rad_, collision_radius_;
  int motor_cmd_max_;
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  turtlelib::DiffDrive diff_drive_;
  double x_, y_, theta_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
