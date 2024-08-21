# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Launch combined (teleop, navi, yolo...) demo in gazebo

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    cob_sim_trad_path = os.path.join(
        get_package_share_directory('cob_sim_trad'))

    # load URDF for cob4-25 robot
    xacro_file = os.path.join(cob_sim_trad_path,
                              'urdf',
                              'cob4-25_torso_head.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # prepare for gazebo
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'tricycle'],
                        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # load ros2 controllers
    load_tricycle_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'tricycle_controller'],
        output='screen'
    )

    load_torso_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'torso_controller'],
        output='screen'
    )

    # load twist_mux
    twist_mux_params = os.path.join(get_package_share_directory('cob_sim_trad'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/tricycle_controller/cmd_vel')]
    )

    # load RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    # load teleop for mobile base
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    joy_config = LaunchConfiguration('joy_config', default='xbox')

    teleop_joy = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
            ),
            launch_arguments={'joy_config': joy_config}.items()
    )

    # load teleop for torso joint
    teleop_torso = Node(
        package="cob_sim_trad",
        executable="torso_teleop_node.py",
        prefix='xterm -e',
        output='screen'
    )
    
    # load laser scanner merger to merge three signals
    laser_merger = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_laser_scan_merger'), 'launch'), '/merge_3_scan_cob.launch.py']),
    )
    
    # load standard slam_toolbox
    slam_params_path = os.path.join(get_package_share_directory('cob_sim_trad'),'config','mapper_params_cob_0409.yaml')
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
                    launch_arguments={
                    'params_file': slam_params_path,
                    'use_sim_time': 'true' 
                }.items()
    )
    
    # load nav2 and configuration
    nav2_params_path = os.path.join(get_package_share_directory('cob_sim_trad'),'config','navi_cob_drive.yaml')
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                    launch_arguments={
                    'params_file': nav2_params_path,
                    'use_sim_time': 'true' 
                }.items()
    )
    
    # load Yolo for simulation
    YoloV8 = Node(
        package="yolobot_recognition",
        executable="yolov8_ros2_pt_sim.py",
        prefix='xterm -e',
        output='screen'
    )

    # load behaviortree_lifecycle_node, use command 'ros2 lifecycle set...' to configure and to activate
    BT_lifecycle = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_behavior_tree_example'), 'launch'), '/cob_bt_lifecycle.launch.py']),
                    launch_arguments={
                    'node_enable': 'True',
                    'node_mode': 'cob_sim' ,                                # choose bt_file folder within [cob_sim , cob_robot]
                    'node_behaviortree' : 'sim_test.xml'    # choose the <BT.xml> file to launch BT for user story
                }.items()
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_tricycle_controller],
            )
        ),
        gazebo,
        rviz2,
        node_robot_state_publisher,
        spawn_entity,
        twist_mux,
        teleop_joy,
        laser_merger,
        slam_toolbox,
        load_torso_controller,
        navigation,
        YoloV8,
        BT_lifecycle,
        # teleop_torso
    ])
