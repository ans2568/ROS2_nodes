FROM ros:foxy
RUN apt-get update && apt-get install build-essential pkg-config ros-foxy-geometry-msgs ros-foxy-rclcpp ros-foxy-ament-cmake
WORKDIR /ros2_ws/src
COPY . /ros2_ws/src
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 run teleop_jackal teleop_node"]