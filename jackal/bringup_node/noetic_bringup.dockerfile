FROM ros:noetic

ENV ROS_HOSTNAME=192.168.127.104
ENV ROS_MASTER_URI=http://192.168.127.104:11311

RUN apt-get update && apt-get install python-is-python3 git wget build-essential pkg-config -y
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone https://github.com/jackal/jackal_robot.git
RUN git clone https://github.com/jackal/jackal.git
WORKDIR /catkin_ws
RUN apt-get update && rosdep update
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && rosdep install --from-paths src --ignore-src -r --rosdistro=${ROS_DISTRO} -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && catkin_make
ENV ROS_IP=192.168.127.104
CMD ["/bin/bash", "-c", ". devel/setup.bash && roslaunch jackal_base base.launch"]
