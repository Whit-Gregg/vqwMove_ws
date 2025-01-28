import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_rviz2_config_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_nav_bringup"),
        "params",
        "fuse.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('fuse_rvizconfig')],
    )

    nodes = [
        launch.actions.DeclareLaunchArgument(name='fuse_rvizconfig', default_value=default_rviz2_config_path, description='Absolute path to rviz config file'),
        rviz_node,
    ]

    return LaunchDescription(nodes)
