FROM ros:foxy

RUN apt-get update && apt-get install curl gnupg2 lsb-release python3-colcon-common-extensions python3-argcomplete ros-foxy-cv-bridge -y
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null
RUN apt-get install apt-transport-https
RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
	tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update 
RUN apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev -y
RUN mkdir -p ~/ros2_ws/src
WORKDIR /root/ros2_ws/src
COPY . /root/ros2_ws/src
WORKDIR /root/ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 run realsense_sub realsense_sub"]