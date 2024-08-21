# YCH_ros2_ws
![demo simulation with cob4 model](https://github.com/ipa-ych/YCH_ros2_ws/blob/main/Media/Screenshots/cob_sim_hospital.gif)
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
https://github.com/4am-robotics/cob_common 
2. ros2_laser_scan_merger<br>
Merge 2/3/more laserscanner signals<br>
https://github.com/mich1342/ros2_laser_scan_merger
3. pointcloud_to_laserscan<br>
COnvert pointcloud to laserscan signal<br>
https://github.com/ros-perception/pointcloud_to_laserscan/tree/humble
4. yolobot_recognition<br>
Perform object detection utilizing RGB camera with Yolov8
5. yolov8_msgs<br>
Msgs for Yolov8<br>
https://drive.google.com/drive/folders/1SyyDtQC7LpSIld-jmtkI1qXXDnLNDg6w
6. ros2_behavior_tree_example<br>
Customize behavior tree and BT action for user story here<br>
https://github.com/polymathrobotics/ros2_behavior_tree_example
7. cob_sim_trad<br>
This package contains customized robot description files (origin from cob_common), parameter configuration and launch files for each user story (teleop, slam, navigation, combi_sim)

## How to use
1. colon the repo to workspace_name/src, build and source with
```bash
colcon build && source install/setup.bash
```
2. Launch the demo
- To launch teleop_demo
```bash
ros2 launch cob_sim_trad cob_teleop_sim.launch.py world:='path_to_world_file'
```
>For teleopration (with standard PS4 Joystick), hold trangle to enable base_driver, left joystick to control the base movement (forward/ backward and spin); hold L1 to enable torso, right joystick to control the spin of torso joint
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