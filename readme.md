# Cooperative Control

This workspace contains the ROS 2 packages used for a cooperative transport setup with two PX4 vehicles and a rod payload.

## Workspace Layout

- `src/px4_ros_com` - PX4 ROS 2 support package
- `src/px4_msgs` - PX4 message definitions
- `src/cooperative_transport` - cooperative transport control nodes
- `src/monitor_pose` - pose monitoring and keyboard control tools
- `cooperative_transport_world.sdf` - Gazebo world for the transport scenario

## Build

From the workspace root:

```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Simulation Setup

Copy `cooperative_transport_world.sdf` into the PX4 Gazebo worlds directory:

```bash
cp cooperative_transport_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

Start the PX4 instances in separate terminals:

```bash
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-0.5,2,1" PX_SIM_MODEL=gz_x500 PX4_GZ_WORLD=cooperative_transport_world ./build/px4_sitl_default/bin/px4 -i 0
```

```bash
cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0.5,2,1" PX_SIM_MODEL=gz_x500 PX4_GZ_WORLD=cooperative_transport_world ./build/px4_sitl_default/bin/px4 -i 1
```

Start the Micro XRCE-DDS agent:

```bash
MicroXRCEAgent udp4 -p 8888
```

## ROS Nodes

Start the pose monitor:

```bash
cd ~/cooperative_control && source install/setup.bash && ros2 run monitor_pose monitor
```

Start the keyboard control node:

```bash
cd ~/cooperative_control && source install/setup.bash && ros2 run monitor_pose keyboard_control
```

Bridge the payload pose from Gazebo to ROS:

```bash
ros2 run ros_gz_bridge parameter_bridge /model/rod_payload/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose
```

Run the cooperative transport controller:

```bash
ros2 run cooperative_transport offboard_control_trajectory
```

## Notes

- The control nodes publish to both PX4 instances through `/fmu/in/...` and `/px4_1/fmu/in/...` topics.
- The `cooperative_transport` package depends on `px4_ros_com`, `px4_msgs`, `rclcpp`, `std_msgs`, and `geometry_msgs`.
- If you use a different ROS 2 distribution, replace `<ros-distro>` in the build commands with your installed distro name.
