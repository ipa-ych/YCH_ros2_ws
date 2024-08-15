from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path

import pdb

def generate_launch_description():
    package_dir = Path(get_package_share_directory('ros2_behavior_tree_example'))
    behavior_tree_dir = package_dir / 'behavior_trees'

    # limit choices so we can only have available files
    mode_choices = ["cob_sim",
                    "cob_robot"]    # add new BT folders here
    
    tree_choices = ["combi_test.xml", 
                    "combi_test_with_Navi_sim.xml", 
                    "navi_hospital_sim.xml",
                    "light_mimic_test.xml", 
                    "sim_test.xml", 
                    "simple_srvs_test.xml", 
                    "sound_test.xml"]   # add new BTs here
    
    enable_choices = ["True", "False"]

    node_enable_arg = DeclareLaunchArgument("node_enable", 
                                            default_value="True",
                                            choices=enable_choices,
                                            description="Enable Primary Node in case you want to launch separately")
    
    node_mode_arg = DeclareLaunchArgument("node_mode", 
                                            default_value="cob_sim", 
                                            choices=mode_choices, 
                                            description="Set trees to reactive sequence")


    node_behaviortree_arg = DeclareLaunchArgument("node_behaviortree", 
                                                    default_value="combi_test.xml", 
                                                    choices=tree_choices, 
                                                    description="Set behevior tree file to use desired nodes")


    bt_lifecycle_node = Node(
        condition=IfCondition(LaunchConfiguration("node_enable")),
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='bt_lifecycle_node',
        prefix='xterm -e',
        output='screen',
        parameters=[{
            "rate_hz" : 0.5,        # 0.5 hz works with srvs_play_sound, 1 hz doesn't :( 
            # "num_republish": 5,
            "num_republish": 10,
            "ping_starter" : True,
            "behaviortree_file" : PathJoinSubstitution([str(behavior_tree_dir),
                                                        LaunchConfiguration("node_mode"),
                                                        LaunchConfiguration("node_behaviortree")])
        }]
    )



    return LaunchDescription([
        node_enable_arg,
        node_mode_arg,
        node_behaviortree_arg,
        bt_lifecycle_node
    ])