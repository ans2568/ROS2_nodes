#include "realsense_node.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealsenseNode>());
    rclcpp::shutdown();
    return 0;
  return 0;
}

