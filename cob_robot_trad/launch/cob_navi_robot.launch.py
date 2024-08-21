# Copyright 2024 Fraunhofer IPA
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
#
# Author: Yuzhang Chen

import os
import xacro
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameters_type import ParameterValue


def generate_launch_description():

    xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25-no-arm.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # display robot

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        prefix = 'xterm -e',
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        prefix = 'xterm -e'
    )

    # teleop
    joy_node_config = os.path.join(
        get_package_share_directory('cob_robot_trad'),
        'config',
        'joy_node.yaml'
    )

    teleop_twist_joy_node_config = os.path.join(
        get_package_share_directory('cob_robot_trad'),
        'config',
        'teleop_twist_params.yaml'
    )

    twist_mux_config = os.path.join(
        get_package_share_directory('cob_robot_trad'),
        'config',
        'twist_mux.yaml'
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        prefix = 'xterm -e',
        output='screen',
        name="joy_node",
        parameters=[joy_node_config]
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        prefix = 'xterm -e',
        output='screen',
        name="teleop_twist_joy_node",
        parameters = [teleop_twist_joy_node_config],
        remappings=[
          ("/cmd_vel", "/cmd_vel")]
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        prefix = 'xterm -e',
        output='screen',
        name="twist_mux",
        remappings=[
        ("cmd_vel", "cmd_vel"),
        ("cmd_vel_out", "/base/twist_mux/command_teleop_joy")]
        ,
        parameters = [twist_mux_config]
  )

    slam_toolbox_config = os.path.join(
        get_package_share_directory('cob_robot_trad'),
        'config',
        'slam_toolbox_params.yaml'
        )
    
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        prefix = 'xterm -e',
        output='screen',
        name="slam_toolbox",
        parameters = [slam_toolbox_config]
    )

    nav2_params_path = os.path.join(get_package_share_directory('cob_robot_trad'),'config','navi_robot.yaml')
    navi_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nav2_bringup'), 'launch'), '/navigation_launch.py']),
                    launch_arguments={
                    'params_file': nav2_params_path,
                    'use_sim_time': 'false' 
                }.items()
        )

    nodes_to_start = [
        # joint_state_publisher_gui,
        robot_state_publisher_node,
        rviz_node,
        joy_node,
        teleop_twist_joy_node,
        twist_mux,
        slam_toolbox, 
        navi_launch
    ]



    return LaunchDescription(nodes_to_start)
