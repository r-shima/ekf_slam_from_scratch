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

using namespace std::chrono_literals;

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
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
    joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&Odometry::joint_states_callback, this,
      std::placeholders::_1));
    timer_ = create_wall_timer(
      500ms, std::bind(&Odometry::timer_callback, this));
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    initial_pose_ = create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(
        &Odometry::initial_pose_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    diff_drive_ = turtlelib::DiffDrive{wheel_radius_, track_width_};

    check_params();
  }

private:
  void check_params() {
    if(wheel_radius_ == -1.0 || track_width_ == -1.0 || body_id_ == "" || wheel_left_ == "" ||
       wheel_right_ == "") {
        int num = 1;
        RCLCPP_ERROR(this->get_logger(), "The parameters are not defined");
        throw num;
    }
  }

  void joint_states_callback(const sensor_msgs::msg::JointState & msg) {
    angle_.l = msg.position[0];
    angle_.r = msg.position[1];
    diff_drive_.forward_kinematics(angle_);
  }

  void initial_pose_callback(const std::shared_ptr<nuturtle_control::srv::InitialPose::Request>
    request, std::shared_ptr<nuturtle_control::srv::InitialPose::Response>) {
    config_.x = request->x;
    config_.y = request->y;
    config_.theta = request->theta;
    diff_drive_ = turtlelib::DiffDrive{wheel_radius_, track_width_, config_};
  }

  void timer_callback() {
    t_.header.stamp = this->get_clock()->now();
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

    odom_.header.stamp = this->get_clock()->now();
    odom_.header.frame_id = odom_id_;
    odom_.child_frame_id = body_id_;
    odom_.pose.pose.position.x = diff_drive_.configuration().x;
    odom_.pose.pose.position.y = diff_drive_.configuration().y;
    odom_.pose.pose.position.z = 0.0;

    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();

    tf_broadcaster_->sendTransform(t_);
    odom_pub_->publish(odom_);
  }

  // Declare private variables for the publisher and timer
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_;

  double wheel_radius_, track_width_;
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
  turtlelib::DiffDrive diff_drive_;
  turtlelib::WheelAngle angle_;
  geometry_msgs::msg::TransformStamped t_;
  nav_msgs::msg::Odometry odom_;
  turtlelib::Config config_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Odometry>());
  } catch (int num) {}
  rclcpp::shutdown();
  return 0;
}