#include "laser_avoidance.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserAvoidance>());
    rclcpp::shutdown();
  return 0;
}
