/// \file
/// \brief This node publishes cmd_vel commands to cause the robot to drive in a circle of a
/// specified radius at a specified speed
///
/// PARAMETERS:
///     frequency (int): a fixed rate used to publish cmd_vel commands
/// PUBLISHES:
///     /cmd_vel (geometry_msgs::msg::Twist): the velocity broken into its linear and angular parts
/// SERVERS:
///     control (nuturtle_control::srv::Control): moves the robot in a circle
///     reverse (std_srvs::srv::Empty): reverses the direction of the robot along the arc
///     stop (std_srvs::srv::Empty): stops the robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

/// \brief Provides control for moving the robot in a circle
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
    // Initializes variables for the parameter, publisher, timer, and services
    declare_parameter("frequency", 100);
    frequency_ = get_parameter("frequency").get_parameter_value().get<int>();
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / frequency_),
      std::bind(&Circle::timer_callback, this));
    control_ = create_service<nuturtle_control::srv::Control>(
      "/control",
      std::bind(
        &Circle::control_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    reverse_ = create_service<std_srvs::srv::Empty>(
      "/reverse",
      std::bind(
        &Circle::reverse_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    stop_ = create_service<std_srvs::srv::Empty>(
      "/stop",
      std::bind(
        &Circle::stop_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    flag_ = 1;
  }

private:
  /// \brief Sets the components of a twist so that the robot will move in a circle
  ///
  /// \param request - angular velocity and radius of the arc
  /// \param response - not being used
  /// \returns none
  void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>) {
    flag_ = 1;
    twist_.linear.x = request->radius * request->velocity;
    twist_.angular.z = request->velocity;
  }

  /// \brief Sets the components of a twist to the negative of themselves
  ///
  /// \param request - not being used
  /// \param response - not being used
  /// \returns none
  void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    flag_ = 1;
    twist_.linear.x = -twist_.linear.x;
    twist_.angular.z = -twist_.angular.z;
  }

  /// \brief Sets the components of a twist to zero
  ///
  /// \param request - not being used
  /// \param response - not being used
  /// \returns none
  void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    flag_ = 0;
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_);
  }

  /// \brief Callback function for the timer. Publishes a twist if the flag is set to 1.
  ///
  /// \param none
  /// \returns none
  void timer_callback() {
    if(flag_ == 1) {
        cmd_vel_pub_->publish(twist_);
    }
  }

  // Declare private variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_;

  int frequency_;
  int flag_;
  geometry_msgs::msg::Twist twist_;
};

/// \brief The main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}