FROM ros:foxy
RUN apt-get update && apt-get install build-essential python3-pip pkg-config python3-colcon-common-extensions -y
RUN pip3 install -U argcomplete
WORKDIR /ros2_ws
COPY . /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 run yrl3_v2_ros2_package yrl3_v2_ros2_node"]