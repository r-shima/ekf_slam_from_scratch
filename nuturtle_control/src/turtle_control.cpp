#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;

class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1);
    declare_parameter("collision_radius", -1.0);
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    motor_cmd_per_rad_sec_ = get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ = get_parameter("encoder_ticks_per_rad").get_parameter_value().get<int>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    if(wheel_radius_ == -1.0 || track_width_ == -1.0 || motor_cmd_max_ == -1 ||
       motor_cmd_per_rad_sec_ == -1.0 || encoder_ticks_per_rad_ == -1 ||
       collision_radius_ == -1.0) {
        int num = 0;
        throw num;
    }

    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      500ms, std::bind(&TurtleControl::timer_callback, this));
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg) {
    twist_.w = msg.angular.z;
    twist_.x = msg.linear.x;
    twist_.y = msg.linear.y;

    vel_ = diff_drive_.inverse_kinematics(twist_);
    wheel_commands_.left_velocity = vel_.l / motor_cmd_per_rad_sec_;
    wheel_commands_.right_velocity = vel_.r / motor_cmd_per_rad_sec_;

    if(wheel_commands_.left_velocity > motor_cmd_max_) {
        wheel_commands_.left_velocity = motor_cmd_max_;
    }
    else if(wheel_commands_.left_velocity < -motor_cmd_max_) {
        wheel_commands_.left_velocity = -motor_cmd_max_;
    }
    if(wheel_commands_.right_velocity > motor_cmd_max_) {
        wheel_commands_.right_velocity = motor_cmd_max_;
    }
    else if(wheel_commands_.right_velocity < -motor_cmd_max_) {
        wheel_commands_.right_velocity = -motor_cmd_max_;
    }
  }
  
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg) {
    
  }
  
  void timer_callback() {
    // RCLCPP_INFO_STREAM(get_logger(), "Publishing wheel velocities:" << wheel_commands_.left_velocity << " " << wheel_commands_.right_velocity);
    wheel_cmd_pub_->publish(wheel_commands_);
  }

  // Declare private variables for the publishers, subscribers, and a timer
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double wheel_radius_, track_width_, motor_cmd_per_rad_sec_, collision_radius_;
  int motor_cmd_max_, encoder_ticks_per_rad_;
  turtlelib::Twist2D twist_;
  turtlelib::DiffDrive diff_drive_;
  turtlelib::WheelVelocity vel_;
  nuturtlebot_msgs::msg::WheelCommands wheel_commands_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TurtleControl>());
  } catch (int num) {
    RCLCPP_ERROR(
      std::make_shared<TurtleControl>()->get_logger(),
      "The parameters are not defined");
  }
  rclcpp::shutdown();
  return 0;
}