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
