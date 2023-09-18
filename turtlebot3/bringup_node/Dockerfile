FROM ros:foxy

RUN apt-get update && apt-get install git curl gnupg2 lsb-release python3-colcon-common-extensions python3-argcomplete ros-foxy-dynamixel-sdk ros-foxy-turtlebot3-msgs ros-foxy-hls-lfcd-lds-driver -y
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src
RUN git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
WORKDIR /ros2_ws
COPY . /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
ENV TURTLEBOT3_MODEL=waffle_pi
ENV LDS_MODEL=LDS-01
ENV FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps-profile.xml
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 launch turtlebot3_bringup robot.launch.py"]