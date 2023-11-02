FROM ros:foxy
RUN apt-get update
RUN apt-get install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions \
    git
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-tf2-eigen ros-$ROS_DISTRO-rviz2

WORKDIR /ros2_ws/src
RUN git clone -b ros2-foxy --recurse-submodules https://github.com/ouster-lidar/ouster-ros.git
WORKDIR /ros2_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY driver_params.yaml /ros2_ws/driver_params.yaml
COPY fastrtps-profile.xml /ros2_ws/fastrtps-profile.xml
COPY driver.launch.py /ros2_ws/src/ouster-ros/ouster-ros/launch/driver.launch.py
CMD [ "/bin/bash", "-c", "export FASTRTPS_DEFAULT_PROFILES_FILE=fastrtps-profile.xml && . install/setup.bash && ros2 launch ouster_ros driver.launch.py params_file:=/ros2_ws/driver_params.yaml"]