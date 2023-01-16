#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

class nusim : public rclcpp::Node
{
    public:
        nusim()
        : Node("nusim"),
          timestep_(0)
        {
            declare_parameter("rate", 200);
            int rate_ = get_parameter("rate").get_parameter_value().get<int>();
            publisher_ = create_publisher<std_msgs::msg::UInt64>("timestep", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(1000/rate_), std::bind(&nusim::timer_callback, this));
        }
    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::UInt64();
            message.data = timestep_++;
            RCLCPP_INFO_STREAM(get_logger(), "Publishing: " << message.data);
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        size_t timestep_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<nusim>());
    rclcpp::shutdown();
    return 0;
}