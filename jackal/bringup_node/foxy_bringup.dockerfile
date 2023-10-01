FROM ros:foxy

RUN apt-get update && apt-get install build-essential pkg-config python3-colcon-common-extensions python3-argcomplete python3-vcstools wget git python3-pip -y
RUN apt-get install ros-${ROS_DISTRO}-twist-mux ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-hardware-interface ros-${ROS_DISTRO}-teleop-twist-joy \
	ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-velodyne* ros-${ROS_DISTRO}-diagnostic-aggregator ros-${ROS_DISTRO}-controller-manager* ros-${ROS_DISTRO}-robot-localization \
	ros-${ROS_DISTRO}-joint* ros-${ROS_DISTRO}-interactive-marker-twist-server ros-${ROS_DISTRO}-urg-node ros-${ROS_DISTRO}-robot-upstart ros-${ROS_DISTRO}-imu-filter-madgwick \
	ros-${ROS_DISTRO}-diff-drive-controller ros-${ROS_DISTRO}-microstrain-inertial-driver -y
RUN mkdir -p /ros2_ws/src
COPY . /ros2_ws/
WORKDIR /ros2_ws/src
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git
RUN git clone https://github.com/RoverRobotics-forks/serial-ros2.git
RUN git clone https://github.com/roasinc/mi_ros2.git
WORKDIR /ros2_ws/
RUN apt-get update && rosdep update
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
RUN . install/setup.sh && ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh
RUN vcs import src < dependencies.repos
COPY accessories.launch.py /ros2_ws/src/jackal_robot/launch/accessories.launch.py
COPY bringup.launch.py /ros2_ws/src/jackal_robot/launch/bringup.launch.py
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
COPY control.yaml /ros2_ws/src/jackal/jackal_control/config/control.yaml
CMD ["/bin/bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps-profile.xml && . install/setup.bash && ros2 launch jackal_robot bringup.launch.py"]
