#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "utility.hpp"
#include <chrono>
using namespace std::chrono_literals;
class PointCloudSubscriber : public rclcpp::Node
{
public:
	PointCloudSubscriber()
		: Node("pointcloud_subscriber")
	{
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
		subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"yrl_scan", 10, std::bind(&PointCloudSubscriber::pointCloudCallback, this, std::placeholders::_1));
		msg_callback_active_ = false;
		timer_ = this->create_wall_timer(100ms, std::bind(&PointCloudSubscriber::publishTwist, this));
		dst_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		pass_through_x_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	}

	void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		geometry_msgs::msg::Twist twist_msg;
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*msg, pcl_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		msg_callback_active_ = false;
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		if (cloud->size() > 0)
		{
			voxelize(cloud, *dst_cloud, 0.01);
			passThrough(dst_cloud, *pass_through_x_cloud, "x", 0.15f, 0.8f);
			for (auto point : pass_through_x_cloud->points)
			{
				double angle = atan2(point.y, point.x) * (180.0 / M_PI);
				if (angle < 40 && angle > -40)
				{
					double distance = sqrt(pow(point.x, 2) + pow(point.y, 2));
					if (distance < 1.0)
						msg_callback_active_ = true;
				}
			}
		}
		publishTwist();
	}

	void publishTwist()
	{
		geometry_msgs::msg::Twist twist_msg;
		if (msg_callback_active_)
		{
			twist_msg.angular.z = 0.5;
			twist_msg.linear.x = 0.0;
			publisher_->publish(twist_msg);
		}
		else
		{
			twist_msg.linear.x = 0.25;
			twist_msg.angular.z = 0.0;
			publisher_->publish(twist_msg);
		}
	}

private:
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr dst_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pass_through_x_cloud;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
	bool msg_callback_active_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto pointCloudSubscriber = std::make_shared<PointCloudSubscriber>();
	rclcpp::spin(pointCloudSubscriber);
	rclcpp::shutdown();
	return 0;
}