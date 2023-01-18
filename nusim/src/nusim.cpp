#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"

class Nusim : public rclcpp::Node
{
    public:
        Nusim()
        : Node("nusim"),
          timestep_(0)
        {
            declare_parameter("rate", 200);
            declare_parameter("x0", 0.0);
            declare_parameter("y0", 0.0);
            declare_parameter("theta0", 0.0);
            rate_ = get_parameter("rate").get_parameter_value().get<int>();
            x0_ = get_parameter("x0").get_parameter_value().get<double>();
            y0_ = get_parameter("y0").get_parameter_value().get<double>();
            theta0_ = get_parameter("theta0").get_parameter_value().get<double>();
            publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
            timer_ = create_wall_timer(std::chrono::milliseconds(1000/rate_),
                                       std::bind(&Nusim::timer_callback, this));
            reset_ = create_service<std_srvs::srv::Empty>("~/reset", 
                                                          std::bind(&Nusim::reset_callback,
                                                          this,
                                                          std::placeholders::_1,
                                                          std::placeholders::_2));
            teleport_ = create_service<nusim::srv::Teleport>("~/teleport",
                                                             std::bind(&Nusim::teleport_callback,
                                                             this,
                                                             std::placeholders::_1,
                                                             std::placeholders::_2));
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::UInt64();
            message.data = timestep_++;
            RCLCPP_INFO_STREAM(get_logger(), "Publishing: " << message.data);
            publisher_->publish(message);

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = get_clock()->now();
            t.header.frame_id = "nusim/world";
            t.child_frame_id = "red/base_footprint";
            t.transform.translation.x = x0_;
            t.transform.translation.y = y0_;
            t.transform.translation.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, theta0_);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
        }

        void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
            timestep_ = 0;
        }

        void teleport_callback(const std::shared_ptr<nusim::srv::Teleport::Request> request,
        std::shared_ptr<nusim::srv::Teleport::Response>)
        {
            x0_ = request->x;
            y0_ = request->y;
            theta0_ = request->theta;
        }

        rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
        rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        size_t timestep_;
        int rate_;
        double x0_;
        double y0_;
        double theta0_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nusim>());
    rclcpp::shutdown();
    return 0;
}