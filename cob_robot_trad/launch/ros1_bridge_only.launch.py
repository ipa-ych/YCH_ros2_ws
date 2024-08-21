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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameters_type import ParameterValue
from launch.event_handlers import OnProcessExit, OnExecutionComplete


def generate_launch_description():

    bridge1_config_path = os.path.join(
        get_package_share_directory('cob_robot_trad'),
        'config',
        # 'bridge_only_params.yaml',
        'bridge_join_sim.yaml'
    )

    load_bridge1_params = ExecuteProcess(
        cmd=['rosparam', 'load', bridge1_config_path]
    )

    launch_bridge1 = ExecuteProcess(
        cmd=['ros2', 'run', 'ros1_bridge', 'parameter_bridge']
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnExecutionComplete(
                target_action=load_bridge1_params,
                on_completion=[
                    LogInfo(msg='load ros1_bridge parameter finished'),
                    launch_bridge1]
            )
        ),
        load_bridge1_params
    ]
    

    )
