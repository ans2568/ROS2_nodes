#include "teleop.hpp"

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VelPublisher>());
	rclcpp::shutdown();
  // printf("hello world teleop_jackal package\n");
  return 0;
}
