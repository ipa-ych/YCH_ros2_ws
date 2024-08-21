import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    robotXacroName = 'cob4-25'

    namePackage = 'cob_hardware_config'

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # urdf = os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25-copy.urdf")

    xacro_file= os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","cob4-25_prismatic.urdf.xacro")
    world_file = os.path.join(get_package_share_directory("cob_hardware_config"),"robots","cob4-25","urdf","empty_world.world")
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    robotDescription = xacro.process_file(xacro_file).toxml()
    # robot_description = {'robot_description': doc.toxml()}  

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("gazebo_ros"),'launch', 'gazebo.launch.py'))

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'world': world_file}.items())

    spawnModelNode = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=['-topic','robot_description','-entity', robotXacroName])
    
    nodeRobotStatePublisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robotDescription,
                'use_sim_time': True }]
            )
    
    LaunchDescriptionObject = LaunchDescription()

    LaunchDescriptionObject.add_action(gazeboLaunch)

    LaunchDescriptionObject.add_action(spawnModelNode)
    LaunchDescriptionObject.add_action(nodeRobotStatePublisher)

    return LaunchDescriptionObject
