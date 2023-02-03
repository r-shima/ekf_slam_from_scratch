// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "nuturtlebot_msgs/msg/wheel_commands.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nuturtlebot_msgs/msg/sensor_data.hpp"

// class TurtleControl : public rclcpp::Node
// {
// public:
//   TurtleControl()
//   : Node("turtle_control"),
//     timestep_(0)
//   {
//     wheel_cmd_pub_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);
//     joint_states_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
//     cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
//       "cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, _1));
//     sensor_data_sub_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
//       "sensor_data", 10, std::bind(&TurtleControl::sensor_data_callback, this, _1));
//     timer_ = create_wall_timer(
//       std::chrono::milliseconds(1000 / rate_),
//       std::bind(&TurtleControl::timer_callback, this));
//   }

// private:
//   void cmd_vel_callback() {
//     //
//     return;
//   }
  
//   void sensor_data_callback() {
//     //
//     return;
//   }
  
//   void timer_callback() {
//     //
//     return;
//   }

//   // Declare private variables for the publishers, subscribers, and a timer
//   rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_pub_;
//   rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
//   rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_sub_;
//   rclcpp::TimerBase::SharedPtr timer_;
//   size_t timestep_;
//   int rate_;
// };

// /// \brief The main function
// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::shutdown();
//   return 0;
// }
int main() {
    return 0;
}