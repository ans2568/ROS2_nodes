## Turtlebot3

### bringup_node

This node enables the internal topics and control for the turtlebot3

- Dockerfile
- fastrtps-profile.xml

**How To Usage**
```bash
cd ~/ROS2_nodes/turtlebot3/bringup_node

docker build -t tb3_bringup .
docker run -it -d --network=host --privileged tb3_bringup
```

### camera_node

This node publishes to the sensor_msgs/msg/CompressedImage topic type using a Turtlebot3 camera.

- Dockerfile

Publish Topic
  - `/image_raw` : sensor_msgs/msg/Image
  - `/image_raw/compressed` : sensor_msgs/msg/CompressedImage

**How To Usage**
```bash
cd ~/ROS2_nodes/turtlebot3/camera_node

docker build -t tb3_camera .
docker run -it -d --network=host --privileged tb3_camera
```

### laser_avoidance_node

This node that uses a 2D lidar to avoid obstacles.

- laser_avoidance source code
- Dockerfile
- fastrtps-profile.xml

Subscribe Topic
  - `/scan` : sensor_msgs/msg/LaserScan

Publish Topic
  - `/cmd_vel` : geometry_msgs/msg/Twist

**How To Usage**
```bash
cd ~/ROS2_nodes/turtlebot3/laser_avoidance_node

docker build -t tb3_laser_avoidance_node .
docker run -it -d --network=host --privileged tb3_laser_avoidance_node
```
