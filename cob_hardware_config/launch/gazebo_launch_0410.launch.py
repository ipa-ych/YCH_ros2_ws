import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Initialize Arguments
    robot = LaunchConfiguration("robot")
    pkg_hardware_config = LaunchConfiguration("pkg_hardware_config")
    description_file = LaunchConfiguration("description_file")

    robotXacroName = 'cob4-25'

    namePackage = 'cob_hardware_config'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf = os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25-copy.urdf")

    urdf_file = os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25_torso.urdf")
    xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25_base.urdf.xacro")
    # Error using xacro file
    world_file = os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","empty_world.world")
    # world_file = os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","ball.world")
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)

    # xacro parsing
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    #urdf parsing
    params = {'robot_description': doc.toxml()}
    robot_description = {'robot_description': doc.toxml()}

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"),'launch', 'gazebo.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': world_file}.items())

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=['-topic','robot_description','-entity', robotXacroName])

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(pkg_hardware_config), "robots", robot, "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
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
                on_exit=[load_controller],
            )
        ),
        gazeboLaunch,
        node_robot_state_publisher,
        spawn_entity,
        # joint_state_publisher_node,
        robot_state_publisher_node,
        # rviz_node
    ])