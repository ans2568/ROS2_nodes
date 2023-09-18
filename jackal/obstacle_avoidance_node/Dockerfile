FROM ros:foxy

WORKDIR /ros2_ws
COPY . /ros2_ws/
RUN apt-get update && apt-get install wget zip unzip build-essential pkg-config python3-colcon-common-extensions python3-argcomplete \
	libboost-all-dev libflann-dev libvtk7-dev ros-foxy-pcl-conversions libusb-1.0.0-dev -y
WORKDIR /Downloads
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.13.1.zip
RUN unzip pcl-1.13.1.zip
WORKDIR /Downloads/pcl-pcl-1.13.1/build
RUN cmake .. -DWITH_OPENGL=FALSE -DCMAKE_BUILD_TYPE=Release  -DBUILD_features=OFF -DBUILD_segmentation=OFF -DBUILD_surface=OFF \
	-DBUILD_visualization=OFF -DBUILD_2d=OFF -DBUILD_ml=OFF -DBUILD_tracking=OFF -DBUILD_tools=OFF -DBUILD_visualization=OFF \
	-DBUILD_octree=ON -DBUILD_kdtree=ON -DBUILD_search=ON -DBUILD_sample_consensus=ON
RUN make -j$(nproc) && make install
ENV LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/fastrtps-profile.xml
CMD ["/bin/bash", "-c", "cd /ros2_ws && . install/setup.bash && ros2 run obstacle_avoidance obstacle_avoidance_node"]