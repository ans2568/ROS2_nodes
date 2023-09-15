FROM ros:noetic

ENV ROS_HOSTNAME=localhost
ENV ROS_MASTER_URI=http://localhost:11311

RUN apt-get update && apt-get install ros-noetic-laser-assembler python-is-python3 git wget build-essential pkg-config \
	ros-${ROS_DISTRO}-laser-assembler unzip libboost-dev libflann-dev libvtk7-dev libeigen3-dev libparmetis-dev \
	ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-rviz* \
	libparmetis-dev -y
RUN mkdir -p /catkin_ws/src/
COPY yujin_yrl_v2_package/ /catkin_ws/src/yujin_yrl_v2_package/
WORKDIR /catkin_ws/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make
WORKDIR /Downloads/
RUN wget -O /Downloads/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip
RUN unzip gtsam.zip -d /Downloads/
WORKDIR /Downloads/gtsam-4.0.0-alpha2/build
RUN cmake ..
RUN make install -j$(nproc)
RUN mkdir /catkin_ws/pointcloud
COPY LeGO-LOAM/ /catkin_ws/src/LeGO-LOAM/
COPY yrl_to_cloud/ /catkin_ws/src/yrl_to_cloud/
WORKDIR /Downloads/
RUN rm gtsam.zip
WORKDIR /usr/include/pcl-1.10/pcl/filters/
COPY voxel_grid.h /usr/include/pcl-1.10/pcl/filters/voxel_grid.h
WORKDIR /catkin_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make -DCATKIN_WHITELIST_PACKAGES="lego_loam;cloud_msgs"
RUN chmod +x src/yrl_to_cloud/src/yrl2pc.py
RUN echo '. /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc
CMD ["/bin/bash", "-c", "cd /catkin_ws && . devel/setup.bash && roslaunch yrl_to_cloud assemble.launch"]