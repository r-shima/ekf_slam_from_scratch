/// \file
/// \brief This node displays a red turtlebot in a simulated environment with cylindrical obstacles
/// and walls. It also allows the robot to move according to simulated kinematics.
///
/// PARAMETERS:
///     rate (int): a frequency used to run the main loop
///     x0 (double): the x component of the initial pose of the robot
///     y0 (double): the y component of the initial pose of the robot
///     theta0 (double): the theta component of the initial pose of the robot
///     obstacles.x (std::vector<double>): a list of the obstacles' x coordinates
///     obstacles.y (std::vector<double>): a list of the obstacles' y coordinates
///     obstacles.r (double): the radius of the obstacles
///     walls.x_length (double): the length of the arena in the world x direction
///     walls.y_length (double): the length of the arena in the world y direction
///     wheel_radius (double): the radius of the wheels
///     track_width (double): the distance between the wheels
///     motor_cmd_max (int): the max motor command
///     motor_cmd_per_rad_sec (double): the motor command "tick"
///     encoder_ticks_per_rad (double): the number of encoder "ticks" per radian
///     collision_radius (double): radius used for collision detection
/// PUBLISHES:
///     /nusim/timestep (std_msgs::msg::UInt64): the timestep for the simulation
///     /nusim/obstacles (visualization_msgs::msg::MarkerArray): cylinderical markers that act as
///                                                              the obstacles
///     /nusim/walls (visualization_msgs::msg::MarkerArray): walls used to create an arena
///     /red/sensor_data (nuturtlebot_msgs::msg::SensorData): sensor data
/// SUBSCRIBES:
///     /red/wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): left and right wheel velocity in
///                                                           "motor command units"
/// SERVERS:
///     reset (std_srvs::srv::Empty): restores the initial state of the simulation and the robot's
///                                   initial location
///     teleport (nusim::srv::Teleport): enables moving the robot to a desired (x, y, theta) pose

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

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
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

std::mt19937 & get_random()
{
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number generator object. This is always the
     // same object every time get_random is called
     return mt;
}

/// \brief Creates a simulation environment for the turtlebot
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"),
    timestep_(0)
  {
    // Initializes variables for the parameters, publishers, subscriber, timer, services, and
    // broadcaster
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
    declare_parameter("input_noise", 0.01);
    declare_parameter("slip_fraction", 0.01);
    declare_parameter("basic_sensor_variance", 0.01);
    declare_parameter("max_range", 10.0);
    declare_parameter("lidar_min_range", 0.11999999731779099);
    declare_parameter("lidar_max_range", 3.5);
    declare_parameter("lidar_angle_increment", 0.01745329238474369);
    declare_parameter("lidar_num_of_samples", 360);
    declare_parameter("lidar_resolution", 1);
    declare_parameter("lidar_noise_variance", 0.01);
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
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();
    input_noise_ = get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction_ = get_parameter("slip_fraction").get_parameter_value().get<double>();
    basic_sensor_variance_ = get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range_ = get_parameter("max_range").get_parameter_value().get<double>();
    lidar_min_range_ = get_parameter("lidar_min_range").get_parameter_value().get<double>();
    lidar_max_range_ = get_parameter("lidar_max_range").get_parameter_value().get<double>();
    lidar_angle_increment_ = get_parameter("lidar_angle_increment").get_parameter_value().get<double>();
    lidar_num_of_samples_ = get_parameter("lidar_num_of_samples").get_parameter_value().get<int>();
    lidar_resolution_ = get_parameter("lidar_resolution").get_parameter_value().get<int>();
    lidar_noise_variance_ = get_parameter("lidar_noise_variance").get_parameter_value().get<double>();
    timestep_pub_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    wall_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", 10);
    sensor_data_pub_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("red/sensor_data", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    fake_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/fake_sensor", 10);
    laser_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/laser_scan", 10);
    wheel_cmd_sub_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(
        &Nusim::wheel_cmd_callback, this,
        std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate_),
      std::bind(&Nusim::timer_callback, this));
    timer2_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&Nusim::timer2_callback, this));
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
    dt_ = 1.0 / rate_;

    thickness_ = 0.15;
    position_x_ = {walls_x_length_ / 2 + thickness_ / 2, -walls_x_length_ / 2 - thickness_ / 2,
      0.0, 0.0};
    position_y_ = {0.0, 0.0, walls_y_length_ / 2 + thickness_ / 2,
      -walls_y_length_ / 2 - thickness_ / 2};
    scale_x_ = {thickness_, thickness_, walls_x_length_ + 2.0 * thickness_,
      walls_x_length_ + 2.0 * thickness_};
    scale_y_ = {walls_y_length_, walls_y_length_, thickness_, thickness_};

    left_noise_ = 0.0;
    right_noise_ = 0.0;
    n_dist_ = std::normal_distribution<>{0.0, std::sqrt(input_noise_)};
    u_dist_ = std::uniform_real_distribution<>{-slip_fraction_, slip_fraction_};
    sensor_n_dist_ = std::normal_distribution<>{0.0, std::sqrt(basic_sensor_variance_)};
    lidar_n_dist_ = std::normal_distribution<>{0.0, std::sqrt(lidar_noise_variance_)};

    check_params();
    add_obstacles();
    add_walls();
  }

private:
  /// \brief Callback function for the timer. Broadcasts the transform and publishes sensor data
  /// and marker arrays for the cylindrical obstacles and walls.
  ///
  /// \param none
  /// \returns none
  void timer_callback()
  {
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_++;
    timestep_pub_->publish(message);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    // t.header.stamp.nanosec += 5e7;
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

    temp_angle_.l = new_vel_.l * dt_ * (1.0 + u_dist_(get_random()));
    temp_angle_.r = new_vel_.r * dt_ * (1.0 + u_dist_(get_random()));
    diff_drive_.forward_kinematics(temp_angle_);
    x_ = diff_drive_.configuration().x;
    y_ = diff_drive_.configuration().y;
    theta_ = diff_drive_.configuration().theta;

    angle_.l = prev_angle_.l + (new_vel_.l * dt_);
    angle_.r = prev_angle_.r + (new_vel_.r * dt_);
    sensor_data_.stamp = get_clock()->now();
    sensor_data_.left_encoder = angle_.l * encoder_ticks_per_rad_;
    sensor_data_.right_encoder = angle_.r * encoder_ticks_per_rad_;
    prev_angle_.l = angle_.l;
    prev_angle_.r = angle_.r;

    sensor_data_pub_->publish(sensor_data_);

    if (timestep_ % 100 == 1)
    {
      pose_.header.stamp = get_clock()->now();
      pose_.header.frame_id = "nusim/world";
      pose_.pose.position.x = x_;
      pose_.pose.position.y = y_;
      pose_.pose.position.z = 0.0;
    
      path_.header.stamp = get_clock()->now();
      path_.header.frame_id = "nusim/world";
      path_.poses.push_back(pose_);
    }

    path_pub_->publish(path_);

    check_collision();
  }

  void timer2_callback() {
    add_fake_sensor();
    simulate_lidar();
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

  /// \brief Callback function for the subscriber that subscribes to
  /// nuturtlebot_msgs/msg/WheelCommands. Converts ticks to to rad per second.
  ///
  /// \param msg - WheelCommands object
  /// \returns none
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    if (msg.left_velocity != 0) {
        left_noise_ = n_dist_(get_random());
    }
    if (msg.right_velocity != 0) {
        right_noise_ = n_dist_(get_random());
    }

    new_vel_.l = static_cast<double>(msg.left_velocity) * motor_cmd_per_rad_sec_ + left_noise_;
    new_vel_.r = static_cast<double>(msg.right_velocity) * motor_cmd_per_rad_sec_ + right_noise_;
  }

  /// \brief Checks if the parameters are specified
  ///
  /// \param none
  /// \returns none
  void check_params()
  {
    if (wheel_radius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1 ||
      motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1.0 ||
      collision_radius_ == -1.0)
    {
      throw(std::runtime_error("The parameters are not defined"));
    }
  }

  /// \brief Creates markers for the cylindrical obstacles and adds them to a marker array
  ///
  /// \param none
  /// \returns none
  void add_obstacles()
  {
    const auto marker_array_size = obstacles_x_.size();

    if (obstacles_x_.size() != obstacles_y_.size()) {
      throw(std::runtime_error("Obstacles' x and y coordinate lists do not have the same length"));
    }

    for (size_t i = 0; i < marker_array_size; i++) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "nusim/world";
      marker.header.stamp = get_clock()->now();
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

  /// \brief Creates markers for the walls and adds them to a marker array
  ///
  /// \param none
  /// \returns none
  void add_walls()
  {
    for (int i = 0; i < 4; i++) {
      visualization_msgs::msg::Marker wall_marker;
      wall_marker.header.frame_id = "nusim/world";
      wall_marker.header.stamp = get_clock()->now();
      wall_marker.id = i;
      wall_marker.type = visualization_msgs::msg::Marker::CUBE;
      wall_marker.action = visualization_msgs::msg::Marker::ADD;
      wall_marker.pose.position.x = position_x_.at(i);
      wall_marker.pose.position.y = position_y_.at(i);
      wall_marker.scale.x = scale_x_.at(i);
      wall_marker.scale.y = scale_y_.at(i);
      wall_marker.color.r = 1.0;
      wall_marker.color.g = 0.0;
      wall_marker.color.b = 0.0;
      wall_marker.color.a = 1.0;
      wall_marker.pose.position.z = 0.125;
      wall_marker.scale.z = 0.25;
      wall_array_.markers.push_back(wall_marker);
    }
  }

  void add_fake_sensor()
  {
    turtlelib::Vector2D vec = {diff_drive_.configuration().x, diff_drive_.configuration().y};
    double theta = diff_drive_.configuration().theta;
    turtlelib::Transform2D T_wr = turtlelib::Transform2D(vec, theta);
    turtlelib::Transform2D T_rw = T_wr.inv();
    visualization_msgs::msg::MarkerArray sensor_array;

    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      turtlelib::Vector2D obs_vec{obstacles_x_.at(i), obstacles_y_.at(i)};
      turtlelib::Vector2D v_ro = T_rw(obs_vec);

      visualization_msgs::msg::Marker sensor_marker;
      sensor_marker.header.frame_id = "red/base_footprint";
      sensor_marker.header.stamp = get_clock()->now();
      sensor_marker.header.stamp.nanosec -= 5e7;
      sensor_marker.id = i;
      sensor_marker.type = visualization_msgs::msg::Marker::CYLINDER;

      if (std::sqrt(std::pow(v_ro.x, 2) + std::pow(v_ro.y, 2)) > max_range_) {
        sensor_marker.action = visualization_msgs::msg::Marker::DELETE;
      }
      else {
        sensor_marker.action = visualization_msgs::msg::Marker::ADD;
      }

      sensor_marker.pose.position.x = v_ro.x + sensor_n_dist_(get_random());
      sensor_marker.pose.position.y = v_ro.y + sensor_n_dist_(get_random());
      sensor_marker.pose.position.z = 0.125;
      sensor_marker.scale.x = 2.0 * obstacles_r_;
      sensor_marker.scale.y = 2.0 * obstacles_r_;
      sensor_marker.scale.z = 0.25;
      sensor_marker.color.r = 1.0;
      sensor_marker.color.g = 1.0;
      sensor_marker.color.b = 0.0;
      sensor_marker.color.a = 1.0;
      sensor_array.markers.push_back(sensor_marker);
    }
    fake_sensor_pub_->publish(sensor_array);
  }

  double calculate_distance(double x1, double x2, double y1, double y2)
  {
    double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    return distance;
  }

  void check_collision()
  {
    turtlelib::Vector2D center_to_center, normalized_vec;
    double distance, travel_distance;
    turtlelib::Config config_after;

    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      distance = calculate_distance(diff_drive_.configuration().x, obstacles_x_.at(i),
        diff_drive_.configuration().y, obstacles_y_.at(i));
      if (distance < collision_radius_ + obstacles_r_)
      {
        center_to_center.x = {diff_drive_.configuration().x - obstacles_x_.at(i)};
        center_to_center.y = {diff_drive_.configuration().y - obstacles_y_.at(i)};
        normalized_vec = turtlelib::normalize_vector(center_to_center);
        travel_distance = collision_radius_ + obstacles_r_ -
          calculate_distance(diff_drive_.configuration().x, obstacles_x_.at(i),
          diff_drive_.configuration().y, obstacles_y_.at(i));
        config_after.x = diff_drive_.configuration().x + travel_distance * normalized_vec.x;
        config_after.y = diff_drive_.configuration().y + travel_distance * normalized_vec.y;
        config_after.theta = diff_drive_.configuration().theta;
        diff_drive_.set_configuration(config_after);
      }
    }
  }

  void simulate_lidar()
  {
    laser_scan_.header.stamp = get_clock()->now();
    // laser_scan_.header.stamp.nanosec -= 2e8;
    laser_scan_.header.frame_id = "red/base_scan";
    laser_scan_.angle_min = 0.0;
    laser_scan_.angle_max = 6.2657318115234375;
    laser_scan_.angle_increment = lidar_angle_increment_;
    laser_scan_.time_increment = 0.0; // 0.0005574136157520115;
    laser_scan_.scan_time = 0.20066890120506287;
    laser_scan_.range_min = lidar_min_range_;
    laser_scan_.range_max = lidar_max_range_;
    laser_scan_.ranges.resize(lidar_num_of_samples_);

    double max_x, max_y, slope, alpha, a, b, c, determinant, distance1, distance2;
    turtlelib::Vector2D obs_intersect1, obs_intersect2, wall_intersect;

    for (int i = 0; i < lidar_num_of_samples_; i++) {
      double chosen_distance = 1000.0;
      double min_distance = 1000.0;
      for (size_t j = 0; j < obstacles_x_.size(); j++) {
        max_x = diff_drive_.configuration().x + lidar_max_range_ * cos(i * lidar_angle_increment_ +
          diff_drive_.configuration().theta);
        max_y = diff_drive_.configuration().y + lidar_max_range_ * sin(i * lidar_angle_increment_ + 
          diff_drive_.configuration().theta);
        slope = (max_y - diff_drive_.configuration().y) / (max_x - diff_drive_.configuration().x);
        alpha = diff_drive_.configuration().y - slope * diff_drive_.configuration().x -
          obstacles_y_.at(j);
        a = 1.0 + std::pow(slope, 2);
        b = 2.0 * (alpha * slope - obstacles_x_.at(j));
        c = std::pow(obstacles_x_.at(j), 2) + std::pow(alpha, 2) - std::pow(obstacles_r_, 2);
        determinant = std::pow(b, 2) - 4.0 * a * c;

        if (determinant < 0.0) {
          double wall1_x = walls_x_length_ / 2;
          double wall1_y = slope * (wall1_x - diff_drive_.configuration().x) +
            diff_drive_.configuration().y;
          double wall1_distance = calculate_distance(wall1_x, diff_drive_.configuration().x, wall1_y,
            diff_drive_.configuration().y);
      
          double wall2_x = -walls_x_length_ / 2;
          double wall2_y = slope * (wall2_x - diff_drive_.configuration().x) +
            diff_drive_.configuration().y;
          double wall2_distance = calculate_distance(wall2_x, diff_drive_.configuration().x, wall2_y,
            diff_drive_.configuration().y);

          double wall3_y = walls_y_length_ / 2;
          double wall3_x = (wall3_y - diff_drive_.configuration().y) / slope +
            diff_drive_.configuration().x;
          double wall3_distance = calculate_distance(wall3_x, diff_drive_.configuration().x, wall3_y,
            diff_drive_.configuration().y);

          double wall4_y = -walls_y_length_ / 2;
          double wall4_x = (wall4_y - diff_drive_.configuration().y) / slope +
            diff_drive_.configuration().x;
          double wall4_distance = calculate_distance(wall4_x, diff_drive_.configuration().x, wall4_y,
            diff_drive_.configuration().y);

          if ((wall1_x - diff_drive_.configuration().x) / (max_x - diff_drive_.configuration().x)
            > 0 && (wall1_y - diff_drive_.configuration().y) / (max_y -
            diff_drive_.configuration().y) > 0) {
            if (wall1_distance < chosen_distance) {
                chosen_distance = wall1_distance;
            }
          }
          if ((wall2_x - diff_drive_.configuration().x) / (max_x - diff_drive_.configuration().x)
            > 0 && (wall2_y - diff_drive_.configuration().y) / (max_y -
            diff_drive_.configuration().y) > 0) {
            if (wall2_distance < chosen_distance) {
                chosen_distance = wall2_distance;
            }
          }
          if ((wall3_x - diff_drive_.configuration().x) / (max_x - diff_drive_.configuration().x)
            > 0 && (wall3_y - diff_drive_.configuration().y) / (max_y -
            diff_drive_.configuration().y) > 0) {
            if (wall3_distance < chosen_distance) {
                chosen_distance = wall3_distance;
            }
          }
          if ((wall4_x - diff_drive_.configuration().x) / (max_x - diff_drive_.configuration().x)
            > 0 && (wall4_y - diff_drive_.configuration().y) / (max_y -
            diff_drive_.configuration().y) > 0) {
            if (wall4_distance < chosen_distance) {
                chosen_distance = wall4_distance;
            }
          }
        }
        else if (determinant == 0.0) {
          obs_intersect1.x = -b / (2 * a);
          obs_intersect1.y = slope * (obs_intersect1.x - diff_drive_.configuration().x) +
            diff_drive_.configuration().y;
          if ((obs_intersect1.x - diff_drive_.configuration().x) / (max_x -
            diff_drive_.configuration().x) > 0 && (obs_intersect1.y -
            diff_drive_.configuration().y) / (max_y - diff_drive_.configuration().y) > 0) {
            chosen_distance = calculate_distance(obs_intersect1.x, diff_drive_.configuration().x,
            obs_intersect1.y, diff_drive_.configuration().y);
          }
        }
        else {
          obs_intersect1.x = (-b + std::sqrt(determinant)) / (2.0 * a);
          obs_intersect1.y = slope * (obs_intersect1.x - diff_drive_.configuration().x) +
            diff_drive_.configuration().y;
          obs_intersect2.x = (-b - std::sqrt(determinant)) / (2.0 * a);
          obs_intersect2.y = slope * (obs_intersect2.x - diff_drive_.configuration().x) +
            diff_drive_.configuration().y;

          distance1 = calculate_distance(obs_intersect1.x, diff_drive_.configuration().x,
            obs_intersect1.y, diff_drive_.configuration().y);
          distance2 = calculate_distance(obs_intersect2.x, diff_drive_.configuration().x,
            obs_intersect2.y, diff_drive_.configuration().y);

          if (distance1 < distance2) {
            if ((obs_intersect1.x - diff_drive_.configuration().x) / (max_x -
              diff_drive_.configuration().x) > 0 && (obs_intersect1.y -
              diff_drive_.configuration().y) / (max_y - diff_drive_.configuration().y) > 0) {
              chosen_distance = distance1;
            }
          }
          else {
            if ((obs_intersect2.x - diff_drive_.configuration().x) / (max_x -
              diff_drive_.configuration().x) > 0 && (obs_intersect2.y -
              diff_drive_.configuration().y) / (max_y - diff_drive_.configuration().y) > 0) {
              chosen_distance = distance2;
            }
          }
        }
        if (chosen_distance < min_distance) {
          min_distance = chosen_distance;
        }
      }
      laser_scan_.ranges.at(i) = min_distance + lidar_n_dist_(get_random());
    }
    laser_scan_pub_->publish(laser_scan_);
  }

  // Declare private variables for the publishers, a subscriber, a timer, services, and a
  // broadcaster
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_marker_pub_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Declare private variables
  size_t timestep_;
  int rate_;
  double x0_, y0_, theta0_;
  std::vector<double> obstacles_x_, obstacles_y_;
  double obstacles_r_, walls_x_length_, walls_y_length_;
  visualization_msgs::msg::MarkerArray marker_array_;
  std::vector<double> position_x_, position_y_, scale_x_, scale_y_;
  visualization_msgs::msg::MarkerArray wall_array_;
  double thickness_;
  turtlelib::WheelAngle angle_, prev_angle_, temp_angle_;
  turtlelib::WheelVelocity new_vel_;
  double wheel_radius_, track_width_, motor_cmd_per_rad_sec_, encoder_ticks_per_rad_,
    collision_radius_, input_noise_, slip_fraction_, basic_sensor_variance_, max_range_;
  double lidar_min_range_, lidar_max_range_, lidar_angle_increment_, lidar_noise_variance_;
  int motor_cmd_max_, lidar_num_of_samples_, lidar_resolution_;
  nuturtlebot_msgs::msg::SensorData sensor_data_;
  turtlelib::DiffDrive diff_drive_;
  double x_, y_, theta_, dt_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pose_;
  double left_noise_, right_noise_;
  std::normal_distribution<> n_dist_{0.0, 0.0};
  std::uniform_real_distribution<> u_dist_{0.0, 0.0};
  std::normal_distribution<> sensor_n_dist_{0.0, 0.0};
  sensor_msgs::msg::LaserScan laser_scan_;
  std::normal_distribution<> lidar_n_dist_{0.0, 0.0};
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
