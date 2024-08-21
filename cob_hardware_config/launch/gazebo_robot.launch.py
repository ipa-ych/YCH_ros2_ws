from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameters_type import ParameterValue
import os
import xacro
import subprocess

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            description="Robot name.",
            choices=["cob4-25"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "pkg_hardware_config",
            default_value="cob_hardware_config",
            description="Name of the package that contains the robot configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=[LaunchConfiguration("robot"),".urdf.xacro"],
            description="File containing the robot description.",
        )
    )

    # Initialize Arguments
    robot = LaunchConfiguration("robot")
    pkg_hardware_config = LaunchConfiguration("pkg_hardware_config")
    description_file = LaunchConfiguration("description_file")

    urdf_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25.urdf")
    

    # Launch Gazebo
    # gazebo_node = Node(
    #     package='gazebo_ros',
    #     executable='gazebo',
    #     name='gazebo',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # Load the URDF model into Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', 'my_robot', '-file', urdf_file]
    )

    nodes_to_start = [
        spawn_entity_node,
        
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)