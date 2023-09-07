# ROS2_nodes
ros2 nodes for Information Model

ROS2 version : foxy

**All of these nodes execute their internal operations automatically.**

---
### camera_node

This node publishes and subscribes to the sensor_msgs/msg/CompressedImage topic type using a RealSense camera.

- publisher, subscriber source code
- Dockerfile
```bash
// image publisher
docker build -t image_publisher -f publisher.dockerfile
docker run -it -d --network=host --privileged image_publisher

// image subscriber
docker build -t image_subscriber -f subscriber.dockerfile
docker run -it -d --network=host --privileged image_subscriber
```
---
### teleop_node

This node is for a robot that changes its speed forward and backward every 3 seconds and moves.

- teleop source code
- Dockerfile
```bash
docker build -t teleop_auto .
docker run -it -d --network=host --privileged teleop_auto
```

---
### lidar_node

This node runs the 3D Lidar using the source code provided by [Yujin Lidar](http://lidar.yujinrobot.com/). 

You can find the referenced GitHub repository at https://github.com/yujinrobot/yujin_lidar_v2/tree/main/driver_ros2_foxy_ubuntu2004/yrl3_v2_ros2_package.

- lidar source code
  - referecne : **https://github.com/yujinrobot/yujin_lidar_v2/tree/main/driver_ros2_foxy_ubuntu2004/**
- Dockerfile
```bash
docker build -t lidar .
docker run -it -d --network=host --privileged lidar
```
