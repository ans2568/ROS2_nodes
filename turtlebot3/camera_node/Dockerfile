FROM ros:foxy

RUN apt-get update && apt-get install python3-colcon-common-extensions python3-argcomplete git ros-${ROS_DISTRO}-v4l2-camera libtheora-dev \
	ros-${ROS_DISTRO}-image-transport-plugins libogg-dev libboost-python-dev -y
WORKDIR /ros2_ws/src
RUN git clone --branch ${ROS_DISTRO} https://gitlab.com/boldhearts/ros2_v4l2_camera.git src/v4l2_camera
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 run v4l2_camera v4l2_camera_node"]