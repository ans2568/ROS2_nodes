#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSubscriberNode : public rclcpp::Node
{
public:
    ImageSubscriberNode()
    : Node("image_subscriber_node")
    {
        // Create a subscriber for the compressed image data
        compressed_image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/color/compressed_image", 10,
            std::bind(&ImageSubscriberNode::compressed_image_callback, this, std::placeholders::_1));
    }

private:
    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // Decode the compressed image
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

            if (!image.empty())
            {
				cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

                // Save the image to file
                std::string file_name = "received_image.jpg";
                cv::imwrite(file_name, image);
                RCLCPP_INFO(this->get_logger(), "Received and saved an image: %s", file_name.c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to decode received image.");
            }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber_;
};
