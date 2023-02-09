/// \file
/// \brief This node enables control of the turtlebot via twist messages
///
/// PARAMETERS:
///     wheel_radius (double): the radius of the wheels
///     track_width (double): the distance between the wheels
///     motor_cmd_max (int): the max motor command
///     motor_cmd_per_rad_sec (double): the motor command "tick"
///     encoder_ticks_per_rad (double): the number of encoder "ticks" per radian
///     collision_radius (double): radius used for collision detection
/// PUBLISHES:
///     /wheel_cmd (nuturtlebot_msgs::msg::WheelCommands): left and right wheel velocity in "motor
///                                                        command units"
///     /joint_states (sensor_msgs::msg::JointState): data to describe the state of a set of joints
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs::msg::Twist): velocity broken into its linear and angular parts
///     /sensor_data (nuturtlebot_msgs::msg::SensorData): sensor data

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

/// \brief Uses cmd_vel commands to move the robot and reads the sensor data
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("turtle_control")
  {
    // Initializes variables for the parameters, publishers, and subscribers
    declare_parameter("wheel_radius", -1.0);
    declare_parameter("track_width", -1.0);
    declare_parameter("motor_cmd_max", -1);
    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    declare_parameter("encoder_ticks_per_rad", -1.0);
    declare_parameter("collision_radius", -1.0);
    wheel_radius_ = get_parameter("wheel_radius").get_parameter_value().get<double>();
    track_width_ = get_parameter("track_width").get_parameter_value().get<double>();
    motor_cmd_max_ = get_parameter("motor_cmd_max").get_parameter_value().get<int>();
    motor_cmd_per_rad_sec_ =
      get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    encoder_ticks_per_rad_ =
      get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    collision_radius_ = get_parameter("collision_radius").get_parameter_value().get<double>();

    wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
    joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));
    sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data", 10, std::bind(
        &TurtleControl::sensor_data_callback, this,
        std::placeholders::_1));

    time_flag_ = 1.0;

    check_params();
  }

private:
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

  /// \brief Callback function for the subscriber that subscribes to geometry_msgs/msg/Twist.
  /// Uses the twist to calculate the wheel commands. Also limits the range of wheel commands and
  /// publishes them.
  ///
  /// \param msg - Twist object
  /// \returns none
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    twist_.w = msg.angular.z;
    twist_.x = msg.linear.x;
    twist_.y = msg.linear.y;

    turtlelib::DiffDrive diff_drive_{wheel_radius_, track_width_};
    vel_ = diff_drive_.inverse_kinematics(twist_);
    wheel_commands_.left_velocity = (int32_t)(vel_.l / motor_cmd_per_rad_sec_);
    wheel_commands_.right_velocity = (int32_t)(vel_.r / motor_cmd_per_rad_sec_);

    if (wheel_commands_.left_velocity > motor_cmd_max_) {
      wheel_commands_.left_velocity = motor_cmd_max_;
    } else if (wheel_commands_.left_velocity < -motor_cmd_max_) {
      wheel_commands_.left_velocity = -motor_cmd_max_;
    }
    if (wheel_commands_.right_velocity > motor_cmd_max_) {
      wheel_commands_.right_velocity = motor_cmd_max_;
    } else if (wheel_commands_.right_velocity < -motor_cmd_max_) {
      wheel_commands_.right_velocity = -motor_cmd_max_;
    }

    wheel_cmd_pub_->publish(wheel_commands_);
  }

  /// \brief Callback function for the subscriber that subscribes to
  /// nuturtlebot_msgs/msg/SensorData. Converts positions to radians, calculates change in angle
  /// over time, and publishes joint states.
  ///
  /// \param msg - SensorData object
  /// \returns none
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    if (time_flag_ == 1.0) {
      joint_state_.header.stamp = get_clock()->now();
      joint_state_.position = {0.0, 0.0};
      joint_state_.velocity = {0.0, 0.0};
      time_flag_ = 0.0;
    }

    time_diff_ = (msg.stamp.sec + 1e-9 * msg.stamp.nanosec) -
      (joint_state_.header.stamp.sec + 1e-9 * joint_state_.header.stamp.nanosec);
    auto temp = sensor_msgs::msg::JointState();
    temp.header.stamp = this->get_clock()->now();
    temp.name = {"wheel_left_joint", "wheel_right_joint"};
    temp.position = {msg.left_encoder / encoder_ticks_per_rad_,
      msg.right_encoder / encoder_ticks_per_rad_};
    temp.velocity =
    {((msg.left_encoder / encoder_ticks_per_rad_) - (joint_state_.position.at(0))) / time_diff_,
      ((msg.right_encoder / encoder_ticks_per_rad_) - (joint_state_.position.at(1))) / time_diff_};

    joint_state_.header.stamp = msg.stamp;
    joint_state_.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_state_.position = {msg.left_encoder / encoder_ticks_per_rad_,
      msg.right_encoder / encoder_ticks_per_rad_};
    joint_state_.velocity =
    {((msg.left_encoder / encoder_ticks_per_rad_) - (joint_state_.position.at(0))) / time_diff_,
      ((msg.right_encoder / encoder_ticks_per_rad_) - (joint_state_.position.at(1))) / time_diff_};

    joint_states_pub_->publish(temp);
  }

  // Declare private variables
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double wheel_radius_, track_width_, motor_cmd_per_rad_sec_, encoder_ticks_per_rad_,
    collision_radius_, time_diff_, prev_stamp_, time_flag_;
  int motor_cmd_max_;
  turtlelib::Twist2D twist_;
  turtlelib::WheelVelocity vel_;
  nuturtlebot_msgs::msg::WheelCommands wheel_commands_;
  sensor_msgs::msg::JointState joint_state_;
  std_msgs::msg::Header prev_time_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}
