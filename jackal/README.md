## Jackal

### bringup_node

This node enables the internal topics and control for the jackal

- dependencies.repos
- Dockerfile
- fastrtps-profile.xml

**How To Usage**
```bash
cd ~/ROS2_nodes/jackal/bringup_node

docker build -t jackal_bringup .
docker run -it -d --network=host --privileged -v /dev/jackal:/dev/jackal jackal_bringup
```

### camera_node

This node publishes and subscribes to the sensor_msgs/msg/CompressedImage topic type using a RealSense camera.

- publisher, subscriber source code
- Dockerfile

Publish/Subscribe Topic
  - `/image_raw/compressed` : sensor_msgs/msg/CompressedImage

**How To Usage**
```bash
cd ~/ROS2_nodes/jackal/camera_node

// image publisher
docker build -t jackal_image_publisher -f publisher.dockerfile
docker run -it -d --network=host --privileged jackal_image_publisher

// image subscriber
docker build -t jackal_image_subscriber -f subscriber.dockerfile
docker run -it -d --network=host --privileged jackal_image_subscriber
```

### lidar_node

This node runs the 3D Lidar using the source code provided by [Yujin Lidar](http://lidar.yujinrobot.com/). 

You can find the referenced GitHub repository at https://github.com/yujinrobot/yujin_lidar_v2/tree/main/driver_ros2_foxy_ubuntu2004/yrl3_v2_ros2_package.

- lidar source code
  - referecne : **https://github.com/yujinrobot/yujin_lidar_v2/tree/main/driver_ros2_foxy_ubuntu2004/**
- Dockerfile
- fastrtps-profile.xml

Publish Topic
  - `/yrl_scan` : sensor_msgs/msg/PointCloud2

**How To Usage**
```bash
cd ~/ROS2_nodes/jackal/lidar_node

docker build -t jackal_lidar .
docker run -it -d --network=host --privileged jackal_lidar
```
