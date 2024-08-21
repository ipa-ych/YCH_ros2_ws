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
from launch_ros.parameters_type import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25.urdf.xacro")   # original xacro from Nadia
    xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25-no-arm.urdf.xacro")  # original without both arms
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    # display robot

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        prefix = 'xterm -e'
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        prefix = 'xterm -e',
    )
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

    # teleop using joystick 'xbox'
    teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    joy_config = LaunchConfiguration('joy_config', default= 'xbox')

    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')
        ),
        launch_arguments= {'joy_config': joy_config}.items()
    )

    nodes_to_start = [
        # joint_state_publisher_gui,  # to start jsp GUI when launching this file seperately
        robot_state_publisher_node,
        rviz_node,
        # teleop_joy
    ]



    return LaunchDescription(nodes_to_start)
