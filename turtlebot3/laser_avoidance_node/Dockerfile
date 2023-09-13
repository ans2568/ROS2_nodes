FROM ros:foxy

RUN apt-get update && apt-get install python3-colcon-common-extensions python3-argcomplete \
	ros-${ROS_DISTRO}-geometry-msgs ros-${ROS_DISTRO}-sensor-msgs -y
WORKDIR /ros2_ws/src
COPY . /ros2_ws
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps-profile.xml && . install/setup.bash && ros2 run laser_avoidance laser_avoidance_node"]
