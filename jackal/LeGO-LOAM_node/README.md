# LeGO-LOAM Node
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
