#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <memory>
#include <string>
#include <functional>
#include <chrono>

using namespace std::chrono_literals;

class VelPublisher : public rclcpp::Node
{
	public:
		VelPublisher()
		: Node("velocity_publisher"), count_(0)
		{
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
			timer_ = this->create_wall_timer(10ms, std::bind(&VelPublisher::timer_callback, this));
			start_time_ = std::chrono::steady_clock::now();
		}
	private:
		void timer_callback()
		{
			auto now = std::chrono::steady_clock::now();
			auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
			auto vel = geometry_msgs::msg::Twist();
			vel.linear.x = 0.5 * std::sin(elapsed_time / 3.0);
			RCLCPP_INFO(this->get_logger(), "Publishing: %lf", vel.linear.x);
			publisher_->publish(vel);
		}
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
		std::chrono::steady_clock::time_point start_time_;
		size_t count_;
};