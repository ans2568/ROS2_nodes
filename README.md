# ROS2_nodes
ros/ros2 nodes for Information Model

ROS2 version : foxy
ROS version : noetic

**All of these nodes execute their internal operations automatically.**

## Jackal

### bringup_node

This node enables the internal topics and control for the jackal

- dependencies.repos
- foxy_bringup.dockerfile
- noetic_bringup.dockerfile
- fastrtps-profile.xml

**How To Usage**
```bash
cd ~/ROS2_nodes/jackal/bringup_node

// ROS foxy version
docker build -t jackal_foxy_bringup -f foxy_bringup.dockerfile .
docker run -it -d --network=host --privileged -v /dev/jackal:/dev/jackal jackal_foxy_bringup

// ROS noetic version
docker build -t jackal_noetic_bringup -f noetic_bringup.dockerfile .
docker run -it -d --network=host --privileged -v /dev/jackal:/dev/jackal jackal_noetic_bringup
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

**`note : There is two lidar node. One is Yujin LiDAR and the other is Ouster LiDAR`**

#### Ouster LiDAR

```
it requires network setting(static IP)

IPv4 : 192.168.10.99/24
```

Publish Topic
  - `/ouster/points` : sensor_msgs/msg/PointCloud2
  - `/ouster/scan` : sensor_msgs/msg/LaserScan
  - `/ouster/imu` : sensor_msgs/Imu
  - **`More information please visit github : https://github.com/ros-drivers/ros2_ouster_drivers`**

**How To Usage**
```bash
cd ~/ROS2_nodes/jackal/lidar_node

docker build -t ouster_lidar --rm -f ouster.dockerfile .
docker run -it -d --network=host --privileged ouster_lidar
```

#### Yujin LiDAR
This node runs the 3D Lidar using the source code provided by [Yujin Lidar](http://lidar.yujinrobot.com/). 

```
it requires network setting(static IP)

IPv4 : 192.168.1.250/24
```

##### parameter
  - `lidar_ip : 192.168.1.250`
  - `min_z : -0.2`
  - `max_z : 2.0`
  - `min_y : -35.0`
  - `max_y : 35.0`
  - `min_x : -35.0`
  - `max_x : 35.0`
  - `min_range : 0`
  - `max_range : 35.0`
  - `min_vertical_angle : -40`
  - `max_vertical_angle : 40`
  - `min_horizontal_angle : -180`
  - `max_horizontal_angle : 180`
  - `filter_level : 30.0`
  - `extrinsic_transform(x, y, z, roll. pitch, yaw) : 0, 0, 0.07, 0, 0, 0`
  - `scan_mode : 1`

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

docker build -t yujin_lidar -f yujin.dockerfile .
docker run -it -d --network=host --privileged yujin_lidar
```

### LeGO-LOAM_node

This node is a node that assembles ROS PointCloud2 data and runs LeGO-LOAM, displaying a GUI

ros : noetic

**reference** : https://github.com/yujinrobot/yujin_lidar_v2/tree/main/LeGO_LOAM_ros1_noetic_TestPackage_ubuntu2004

`Note : If you do not want me to build it with a Dockerfile, please refer to the reference link and proceed with it directly.`

## How To Usage
```
mkdir ~/catkin_ws

cd ~/catkin_ws

git clone https://github.com/yujinrobot/yujin_lidar_v2.git

cd ~/ROS2_nodes/jackal/LeGO-LOAM_node

git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git

Replace 'LeGO-LOAM' in ~/ROS2_nodes/jackal/LeGO-LOAM_node/LeGO-LOAM/' with 'LeGO-LOAM' in '~/catkin_ws/yujin_lidar_v2/driver_ros1_noetic_ubuntu2004'

// Assemble LiDAR PointCloud2 and Publish assembled PointCloud2 Node
cd ~/ROS2_nodes/jackal/LeGO-LOAM_node
docker build -t assemble_lidar -f assemble_lidar.dockerfile .
docker run -it -d --network=host --privileged assemble_lidar

// LeGO-LOAM and rviz GUI Node
cd ~/ROS2_nodes/jackal/LeGO-LOAM_node
docker build -t loam -f lego_loam.dockerfile .
docker run -it --network=host --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix loam
```

### Modify utility.h
```
$ cd ~/ROS2_nodes/jackal/LeGO-LOAM/LeGO-LOAM/include
$ gedit utility.h

In line 57,
Change 'fileDirectory' to your directory for saving pcd files

ex) extern const string fileDirectory = "/home/tof-hjkim2/catkin_ws/";
```

---
## Map your environment
**Default scanning mode we support is mode 2, so please change LiDAR's scanning mode to 2 through viewer before trying mapping.
When obtaining point cloud data for mapping, data collection should be carried out by moving 0.5 meters and stopping for 2~3 seconds, and so on.
Also, REMOVE OBJECTS NEAR THE LIDAR to avoid poor quality of mapping.**

**When you start mapping your environment, DO MAP FIRST IN STRAIGHT COURSE. Take enough time for mapping though straight course to get enough data for making the map. After then, map in curved course rotating your robot.**

If you want to use LiDAR scanning mode 1, 3 and 4, 
you should modify the value of 'max_clouds' parameter in ~/catkin_ws/src/yrl_to_cloud/launch/assemble.launch
and values of 'N_SCAN', 'ang_res_y', 'ang_bottom' in ~/catkin_ws/src/LeGO-LOAM/LeGO-LOAM/include/utility.h

'max_clouds': As the vertical field of view increases, the buffer size must be increased. Because, to explore point clouds between consecutive frames for the full range of field of view, proportional amount of point clouds are needed. As 'max_clouds' increase, waiting time for mapping should be also increased. (scanning time of 1 cloud: 0.05 sec, scanning time for 'max_clouds': 0.05 * max_clouds)

'N_SCAN': This variable represents the number of channels for multichannel LiDAR. For YRL3V2 mode 2, 24 is the optimum value. You can also adjust this value in proportion to the size of the vertical field of view.

'ang_res_y': The resolution of the vertical axis, i.e. the y-axis. [Vertical FoV / (Number of channels-1)]

'ang_bottom': The lower vertical angle value for LiDAR's vertical field of view. For example, in case of mode 2, 'ang_bottom' is 5 because vertical field of view is from -5 degrees to 35 degrees.

All parameters mentioned above are carefully optimized for mode 2. If you would like to use other scanning mode, please refer to explanations above and change the variable values.

### Execute yrl_to_cloud package 
```
<Terminal 1>
$ cd ~/catkin_ws/src/yrl_to_cloud/src
$ chmod +x ./yrl2pc.py
$ roslaunch yrl_to_cloud assemble.launch

<Terminal 2>
$ cd ~/catkin_ws/
$ rosbag record /assemble_yrl

When your mapping is done, stop rosbag record.
```
### Run YRL3V2 LeGO-LOAM
```
<Terminal 1> 
$ roslaunch lego_loam run.launch

<Terminal 2>
$ cd ~/catkin_ws/
$ rosbag play < bagfile_name >.bag --clock --topic /assemble_yrl

<Terminal 3> 
$ cd ~/catkin_ws/
$ rosrun pcl_ros pointcloud_to_pcd input:=/laser_cloud_surround

Commands for terminal 3 is optional. If you want to get a map, run them, then you will get pcd files created (in your path set in utility.h) after LeGo-LOAM is finished. (ex. cornerMap.pcd, finalCloud.pcd, surfaceMap.pcd, trajectory.pcd and etc.)
```

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

## Common

### teleop_node

This node is for a robot that changes its speed forward and backward every 3 seconds and moves.

- teleop source code
- Dockerfile

Publish Topic
  - `/cmd_vel` : geometry_msgs/msg/Twist

**How To Usage**
```bash
cd ~/ROS2_nodes/common/teleop_node

docker build -t teleop_auto .
docker run -it -d --network=host --privileged teleop_auto
```
