#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

class LaserAvoidance : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

        void move(geometry_msgs::msg::Twist &msg, float linear_x, float angular_z)
        {
            msg.linear.x = linear_x;
            msg.angular.z = angular_z;
        }

        void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            double min_distance = 10;
            int index = 0;
            geometry_msgs::msg::Twist twist_msg;
            for (int i = 0; i < int(msg->ranges.size() - 1); i++)
            {
                auto distance = msg->ranges[i];
                if (distance > 0 && min_distance > distance)
                {
                    min_distance = distance;
                    index = i;
                }
            }
            // forward
            if (min_distance < 0.75 && (index < 70 || index > 290))
            {
                std::cout << "Find a obstacle from forward direction. angular.z to 0.2" << std::endl;
                move(twist_msg, 0.0, 0.2);
            }
            else
            {
                std::cout << "There are no obstacles in the direction of travel" << std::endl;
                move(twist_msg, 0.15, 0);
            }
            std::cout << "min_angle : " << index << std::endl;
            std::cout << "min_distance : " << min_distance << std::endl;
            twist_pub_->publish(twist_msg);
        }

    public:
        LaserAvoidance() : Node("laser_avoidance_node")
        {
            laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::SensorDataQoS(), std::bind(&LaserAvoidance::laserScanCallback, this, std::placeholders::_1));
            twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
};