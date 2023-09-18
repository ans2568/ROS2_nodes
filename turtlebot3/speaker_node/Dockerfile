FROM ros:foxy

RUN apt-get update && apt-get install python3-pip alsa-utils ffmpeg -y
RUN pip install pydub
RUN mkdir -p /ros2_ws/src
COPY ./src /ros2_ws/src
COPY ./human_detect.wav /ros2_ws
WORKDIR /ros2_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build
CMD ["/bin/bash", "-c", ". install/setup.bash && ros2 run speaker speaker_node"]