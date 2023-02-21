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

using namespace std::chrono_literals;

/// \brief Computes the odometry of the robot
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Initializes variables for the parameters, publisher, subscriber, timer, service, and
    // broadcaster
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("body_id", "");
    declare_parameter("odom_id", "odom");
    declare_parameter("wheel_left", "");
    declare_parameter("wheel_right", "");
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("blue/path", 10);
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(
        &Odometry::joint_states_callback, this,
        std::placeholders::_1));
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    initial_pose_ = create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(
        &Odometry::initial_pose_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    prev_angle_.position = {0.0, 0.0};

    diff_drive_ = turtlelib::DiffDrive{wheel_radius_, track_width_};

    timestep_ = 0;

    check_params();
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

    turtlelib::Twist2D twist = diff_drive_.angles_to_twist(angle_);
    odom_.header.stamp = get_clock()->now();
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    odom_.pose.pose.position.x = diff_drive_.configuration().x;
    odom_.pose.pose.position.y = diff_drive_.configuration().y;
    odom_.pose.pose.position.z = 0.0;
    odom_.twist.twist.linear.x = twist.x;
    odom_.twist.twist.angular.z = twist.w;

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

  // Declare private variables
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_;

  double wheel_radius_, track_width_;
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  turtlelib::DiffDrive diff_drive_;
  turtlelib::WheelAngle angle_;
  geometry_msgs::msg::TransformStamped t_;
  nav_msgs::msg::Odometry odom_;
  turtlelib::Config config_;
  sensor_msgs::msg::JointState prev_angle_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped pose_;
  int timestep_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}
