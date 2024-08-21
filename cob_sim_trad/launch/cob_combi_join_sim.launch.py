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

# Join simulation using both physical robot and Gazebo simulator
# Navigation runs in simulator, rest functionalities on physical robot

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

    xacro_file = os.path.join(cob_sim_trad_path,
                              'urdf',
                              'cob4-25_torso_head.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

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

    twist_mux_params = os.path.join(get_package_share_directory('cob_sim_trad'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/tricycle_controller/cmd_vel')]
        )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )


    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    joy_config = LaunchConfiguration('joy_config', default='xbox')

    teleop_joy = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
            ),
            launch_arguments={'joy_config': joy_config}.items()
        )
    
    laser_merger = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_laser_scan_merger'), 'launch'), '/merge_3_scan_cob.launch.py']),
             )
    
    slam_params_path = os.path.join(get_package_share_directory('cob_sim_trad'),'config','mapper_params_cob_0409.yaml')
    slam_toolbox = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('slam_toolbox'), 'launch'), '/online_async_launch.py']),
                    launch_arguments={
                    'params_file': slam_params_path,
                    'use_sim_time': 'true' 
                }.items()
        )

#     slam_toolbox = Node(
#     package="slam_toolbox",
#     executable="async_slam_toolbox_node",
#     output='screen',
#     name="slam_toolbox",
#     parameters = [slam_params_path,
#                   {'use_sim_time': 'true'}]
#   )
    
    nav2_params_path = os.path.join(get_package_share_directory('cob_sim_trad'),'config','navi_speedup.yaml')
    navigation = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                    launch_arguments={
                    'params_file': nav2_params_path,
                    'use_sim_time': 'true' 
                }.items()
        )
    
    image_transport = Node(
            package='image_transport',
            executable='republish',
            name='image_transport_republish',
            output='screen',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', '/torso_cam3d_left_upright/rgb/image_raw/compressed'),
                ('out', '/cam_left/decompressed')
            ]
    )

    YoloV8 = Node(
        package="yolobot_recognition",
        executable="yolov8_ros2_pt.py",
        prefix='xterm -e',
        output='screen'
    )
    
    YoloV8_sim = Node(
        package="yolobot_recognition",
        executable="yolov8_ros2_pt_sim.py",
        prefix='xterm -e',
        output='screen'
    )

    BT_lifecycle = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros2_behavior_tree_example'), 'launch'), '/cob_bt_lifecycle.launch.py']),
                    launch_arguments={
                    'node_enable': 'True',
                    'node_mode': 'cob_sim' ,                        # [cob_sim , cob_robot]
                    'node_behaviortree' : 'combi_test_with_Navi_sim.xml'    # choose the BT.xml file to
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
        # YoloV8_sim,
        image_transport,
        YoloV8,
        BT_lifecycle,
    ])
