/// \file
/// \brief This node publishes odometry messages and the odometry transform
///
/// PARAMETERS:
///     wheel_radius (double): the radius of the wheels
///     track_width (double): the distance between the wheels
///     body_id (std::string): the name of the body frame of the robot
///     odom_id (std::string): the name of the odometry frame
///     wheel_left (std::string): the name of the left wheel joint
///     wheel_right (std::string): the name of the right wheel joint
/// PUBLISHES:
///     /odom (nav_msgs::msg::Odometry): an estimate of a position and velocity in free space
/// SUBSCRIBES:
///     /joint_states (sensor_msgs::msg::JointState): data to describe the state of a set of joints
/// SERVERS:
///     initial_pose (nuturtle_control::srv::InitialPose): resets the location of the odometry

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/ekf.hpp"

using namespace std::chrono_literals;

/// \brief Computes the odometry of the robot
class Slam : public rclcpp::Node
{
public:
  Slam()
  : Node("slam")
  {
    // Initializes variables for the parameters, publisher, subscriber, timer, service, and
    // broadcaster
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("green_body_id", "green/base_footprint");
    declare_parameter("green_odom_id", "green/odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    declare_parameter("obstacles.r", 0.038);
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    body_id_ = get_parameter("green_body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("green_odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    slam_obs_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("slam_obstacles", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &Slam::joint_states_callback, this,
        std::placeholders::_1));
    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      "/nusim/fake_sensor", 10, std::bind(
        &Slam::fake_sensor_callback, this,
        std::placeholders::_1));
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster2_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    initial_pose_ = create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(
        &Slam::initial_pose_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    prev_angle_.position = {0.0, 0.0};

    diff_drive_ = turtlelib::DiffDrive{wheel_radius_, track_width_};

    timestep_ = 0;

    check_params();
    create_slam_obstacles();
  }

private:
  /// \brief Checks if the parameters are specified
  ///
  /// \param none
  /// \returns none
  void check_params()
  {
    if (wheel_radius_ == -1.0 || track_width_ == -1.0 || body_id_ == "" || wheel_left_ == "" ||
      wheel_right_ == "")
    {
      throw(std::runtime_error("The parameters are not defined"));
    }
  }

  /// \brief broadcast the transform between odom and body
  ///
  /// \param none
  /// \returns none
  void broadcast_odom_to_body()
  {
    t_.header.stamp = get_clock()->now();
    t_.header.frame_id = odom_id_;
    t_.child_frame_id = body_id_;
    t_.transform.translation.x = diff_drive_.configuration().x;
    t_.transform.translation.y = diff_drive_.configuration().y;
    t_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, diff_drive_.configuration().theta);
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t_);
  }
  
  /// \brief broadcast the transform between map and odom
  ///
  /// \param none
  /// \returns none
  void broadcast_map_to_odom()
  {
    turtlelib::Transform2D T_or{turtlelib::Vector2D{diff_drive_.configuration().x,
      diff_drive_.configuration().y}, diff_drive_.configuration().theta};
    turtlelib::Transform2D T_mo = T_mr * T_or.inv();

    map_odom_.header.stamp = get_clock()->now();
    map_odom_.header.frame_id = "map";
    map_odom_.child_frame_id = odom_id_;
    map_odom_.transform.translation.x = T_mo.translation().x;
    map_odom_.transform.translation.y = T_mo.translation().y;
    map_odom_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, T_mo.rotation());
    map_odom_.transform.rotation.x = q.x();
    map_odom_.transform.rotation.y = q.y();
    map_odom_.transform.rotation.z = q.z();
    map_odom_.transform.rotation.w = q.w();

    tf_broadcaster2_->sendTransform(map_odom_);
  }

  /// \brief Callback function for the subscriber that subscribes to sensor_msgs/msg/JointState.
  /// Updates the internal odometry state, broadcasts the transform, and publishes odometry.
  ///
  /// \param msg - JointState object
  /// \returns none
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    angle_.l = msg.position.at(0) - prev_angle_.position.at(0);
    angle_.r = msg.position.at(1) - prev_angle_.position.at(1);
    diff_drive_.forward_kinematics(angle_);

    broadcast_odom_to_body();
    broadcast_map_to_odom();

    twist_ = diff_drive_.angles_to_twist(angle_);
    odom_.header.stamp = get_clock()->now();
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    odom_.pose.pose.position.x = diff_drive_.configuration().x;
    odom_.pose.pose.position.y = diff_drive_.configuration().y;
    odom_.pose.pose.position.z = 0.0;
    odom_.twist.twist.linear.x = twist_.x;
    odom_.twist.twist.angular.z = twist_.w;

    tf2::Quaternion q;
    q.setRPY(0, 0, diff_drive_.configuration().theta);
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom_);

    prev_angle_.position = {msg.position.at(0), msg.position.at(1)};

    timestep_++;
    if (timestep_ % 100 == 1)
    {
      pose_.header.stamp = get_clock()->now();
      pose_.header.frame_id = odom_id_;
      pose_.pose.position.x = diff_drive_.configuration().x;
      pose_.pose.position.y = diff_drive_.configuration().y;
      pose_.pose.position.z = 0.0;
    
      path_.header.stamp = get_clock()->now();
      path_.header.frame_id = odom_id_;
      path_.poses.push_back(pose_);
    }

    path_pub_->publish(path_);
    slam_obs_pub_->publish(slam_obs_array_);
  }

  /// \brief Callback function for the initial_pose. Resets the location of the odometry.
  ///
  /// \param request - x, y, and theta components of the initial pose
  /// \param response - not being used
  /// \returns none
  void initial_pose_callback(
    const std::shared_ptr<nuturtle_control::srv::InitialPose::Request>
    request, std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    config_.x = request->x;
    config_.y = request->y;
    config_.theta = request->theta;
    diff_drive_ = turtlelib::DiffDrive{wheel_radius_, track_width_, config_};
  }

  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray & msg) {
    ekf_.predict(turtlelib::Twist2D{diff_drive_.configuration().theta,
      diff_drive_.configuration().x, diff_drive_.configuration().y});

    visualization_msgs::msg::MarkerArray landmarks = msg;
    for (size_t i = 0; i < landmarks.markers.size(); i++)
    {
      if (landmarks.markers.at(i).action < 2)
      {
        ekf_.update(landmarks.markers.at(i).pose.position.x,
          landmarks.markers.at(i).pose.position.y, i);
      }
    }
    turtlelib::Config green_robot = ekf_.get_configuration();
    T_mr = turtlelib::Transform2D{turtlelib::Vector2D{green_robot.x, green_robot.y},
      green_robot.theta};
  }

  void create_slam_obstacles() {
    arma::mat xi = ekf_.get_xi();
    int n = ekf_.get_n();

    for (int i = 0; i < n; i++) {
      double slam_obs_x = xi(2*i+3, 0);
      double slam_obs_y = xi(2*i+4, 0);

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = get_clock()->now();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = slam_obs_x;
      marker.pose.position.y = slam_obs_y;
      marker.pose.position.z = 0.125;
      marker.scale.x = 2.0 * obstacles_r_;
      marker.scale.y = 2.0 * obstacles_r_;
      marker.scale.z = 0.25;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      slam_obs_array_.markers.push_back(marker);
    }
  }

  // Declare private variables
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slam_obs_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_, tf_broadcaster2_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_;

  double wheel_radius_, track_width_, obstacles_r_;
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  turtlelib::DiffDrive diff_drive_;
  turtlelib::WheelAngle angle_;
  geometry_msgs::msg::TransformStamped t_, map_odom_;
  nav_msgs::msg::Odometry odom_;
  turtlelib::Config config_;
  sensor_msgs::msg::JointState prev_angle_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pose_;
  int timestep_;
  turtlelib::Twist2D twist_;
  turtlelib::EKF ekf_;
  turtlelib::Transform2D T_mr;
  visualization_msgs::msg::MarkerArray slam_obs_array_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}
