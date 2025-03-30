import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    ld = LaunchDescription()

    container = ComposableNodeContainer(
            name='joy_pan_tilt_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='joy_pan_tilt',
                    plugin='joy_pan_tilt::JoyPanTilt',
                    name='JoyPanTilt',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    ),
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'INFO']  
    )
    ld.add_action(container)
    
    return ld