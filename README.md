# YCH_ros2_ws
![demo simulation with cob4 model](https://github.com/ipa-ych/YCH_ros2_ws/blob/main/Media/Screenshots/cob_sim_hospital.gif)<br>
This package contains subsystems and components for simulation utilizing Care-O-bot 4 (mobile robot combining navigation, detection and decision making with Behavior Tree) in Gazebo 11, implemented with ROS2 Humble

## Prerequisite
1. ROS2 (Tested on Humble)
2. Gazebo (Tested on Gazebo 11.10.2)
3. RViz2
4. ros1_bridge (for tests with physical robot)
5. PS4 joystick (for remote control)
6. Behavior Tree (to use behavior tree) https://www.behaviortree.dev/

## Content
1. cob_common<br>
This package contains communication interfaces (actions, msgs, srvs) and robot description of Care-O-bot 4<br>
Origin: https://github.com/4am-robotics/cob_common 
2. ros2_laser_scan_merger<br>
Merge 2/3/more laserscanner signals<br>
Origin: https://github.com/mich1342/ros2_laser_scan_merger
3. pointcloud_to_laserscan<br>
Convert pointcloud to laserscan signal<br>
Origin: https://github.com/ros-perception/pointcloud_to_laserscan/tree/humble
4. yolobot_recognition<br>
Perform object detection utilizing RGB camera with Yolov8
5. yolov8_msgs<br>
This package contains Msgs for Yolov8<br>
Origin: https://drive.google.com/drive/folders/1SyyDtQC7LpSIld-jmtkI1qXXDnLNDg6w
6. ros2_behavior_tree_example<br>
Customize behavior tree and BT action for user story here<br>
Origin: https://github.com/polymathrobotics/ros2_behavior_tree_example
7. cob_sim_trad<br>
This package contains customized robot description files (origin from cob_common), parameter configuration and launch files for each user story in simulation (teleop, slam, navigation, combi_sim)
8. cob_hardware_config<br>
This package contains URDF and configuration for Care-O-bot model<br>
Origin: https://github.com/ipa320/cob_robots/tree/humble_dev
9. cob_calibration_data<br>
This package contains calibration data for Care-O-bot serie<br>
Origin: https://github.com/ipa-nhg/cob_calibration_data/tree/humble_dev
10. cob_robot_trad<br>
This package contains parameter configuration and launch files for each user story with physical Care-O-bot 4 (teleop, slam, navigation, combi_sim)

## How to use - Simulation
1. Colon the repo to workspace_name/src, build and source with
```bash
colcon build && source install/setup.bash
```
2. Launch the demo
- To launch teleop_demo
```bash
ros2 launch cob_sim_trad cob_teleop_sim.launch.py world:='path_to_world_file'
```
>For teleopration (with standard PS4 Joystick), hold trangle to enable base_driver, left joystick to control the base movement (forward/ backward and spin); hold L1 to enable torso, right joystick to control the spin of torso joint<br>
![Remote Controll](https://github.com/ipa-ych/YCH_ros2_ws/blob/main/Media/Screenshots/remote_settings.png)<br>
- To launch SLAM_demo
```bash
ros2 launch cob_sim_trad cob_slam_sim.launch.py world:='path_to_world_file'
```
- To launch navigation_demo
```bash
ros2 launch cob_sim_trad cob_navi_sim.launch.py world:='path_to_world_file'
```
- To launch combi_demo (with object detection and BT)
```bash
ros2 launch cob_sim_trad cob_combi_sim.launch.py world:='path_to_world_file'
```
> In another Terminal, configurate and activate the behaviortree_lifecycle_node with 
```bash
ros2 lifecycle set /bt_lifecycle_node configure
ros2 lifecycle set /bt_lifecycle_node activate
```

## How to use - Test with physical Care-O-bot 4
1. Colon the repo to workspace_name/src, build and source with
```bash
colcon build && source install/setup.bash
```
> make sure the dependencies are installed: [cob_calibration_data](https://github.com/ipa-nhg/cob_calibration_data/tree/humble_dev)
, [cob_hardware_config](https://github.com/ipa320/cob_robots/tree/humble_dev)...
2. Robot bringup
- Open new termial, configure ROS_MASTER_URI with
```bash
export ROS_MASTER_URI=<robot_address>
```
- Bringup physical robot with
```bash
roslaunch cob_bringup robot.launch  #robot bringup with base, torso and head
```
or with
```bash
roslaunch cob_bringup base_solo.launch  #only the base
```
3. Launch ros1_bridge
- install ros1_bridge and configure environment following [ros1_bridge](https://github.com/ros2/ros1_bridge)
- configure the bridge settings within file /cob_robot_trad/config/bridge1_params.yaml; multiple bridges could be launched simultaneously
- open new termial, configure ROS_MASTER_URI with
```bash
export ROS_MASTER_URI=<robot_address>
```
- in the same terminal, launch bridge with
```bash
ros2 launch cob_robot_trad ros1_bridges.launch.py
```
4. Launch the demo
- open new terminal, launch demo with
```bash
ros2 launch cob_robot_trad cob_combi_robot.launch.py
```