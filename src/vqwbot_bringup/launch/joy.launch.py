#
# publish_stamped_twist
# require_enable_button
# enable_button
# enable_turbo_button
# axis_linear
# axis_angular
# scale_linear
# scale_linear_turbo
# scale_angular
# scale_angular_turbo
#
import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    joy_params_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_bringup"),
        "params",
        "joy_params.yaml"
    ])
    
    ld = LaunchDescription()
    
    game_controller_node = Node(
        package="joy",
        executable="game_controller_node",
        name="game_controller_node",
        parameters=[joy_params_path],
        # remappings=[],
        output="both",
    )
    ld.add_action(game_controller_node)
    

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_params_path],
        # remappings=[],
        output="both",
    )
    ld.add_action(teleop_twist_joy_node)
    

    # # teleop_joy_launch = IncludeLaunchDescription(
    # #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('teleop_twist_joy'), 'launch', 'teleop-launch.py')])
    # #     ,launch_arguments = {
    # #         'config_filepath' : joy_params_path,
    # #         'joy_dev' : '0',
    # #         'joy_vel' : 'cmd_vel',
    # #         'publish_stamped_twist' : 'true',
    # #     }.items(),
    # # )
    # # ld.add_action(teleop_joy_launch)
    
    return ld
    