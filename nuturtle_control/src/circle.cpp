#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {
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
  void control_callback(const std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>) {
    flag_ = 1;
    twist_.linear.x = request->radius * request->velocity;
    twist_.angular.z = request->velocity;
  }

  void reverse_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    flag_ = 1;
    twist_.linear.x = -twist_.linear.x;
    twist_.angular.z = -twist_.angular.z;
  }

  void stop_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    flag_ = 0;
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_);
  }

  void timer_callback() {
    if(flag_ == 1) {
        cmd_vel_pub_->publish(twist_);
    }
  }

  // Declare private variables for the publisher and timer
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