#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

class RealsenseNode : public rclcpp::Node
{
public:
    RealsenseNode()
    : Node("realsense_node")
    {
        // Initialize the Realsense pipeline
        pipeline_.start();

        // Create a publisher for the compressed image data
        compressed_image_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/color/compressed_image", 10);

        // Set up a timer to publish images periodically
        timer_ = this->create_wall_timer(10ms, std::bind(&RealsenseNode::publish_compressed_image, this));
    }

private:
    void publish_compressed_image()
    {
        rs2::frameset frames = pipeline_.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();

        if (color_frame)
        {
            // Convert Realsense frame to OpenCV Mat
            cv::Mat image(cv::Size(color_frame.as<rs2::video_frame>().get_width(),
                                   color_frame.as<rs2::video_frame>().get_height()),
                          CV_8UC3, const_cast<void *>(color_frame.get_data()), cv::Mat::AUTO_STEP);

            // Compress the image using JPEG format
            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(80); // JPEG quality (0-100)

            std::vector<uchar> compressed_data;
            cv::imencode(".jpg", image, compressed_data, compression_params);

            // Create CompressedImage message
            auto compressed_image_msg = sensor_msgs::msg::CompressedImage();
            compressed_image_msg.format = "jpeg";
            compressed_image_msg.data = compressed_data;
            std::cout << "publish color frame" << std::endl;
            compressed_image_publisher_->publish(compressed_image_msg);
        }
    }
    rs2::pipeline pipeline_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_publisher_;
};
