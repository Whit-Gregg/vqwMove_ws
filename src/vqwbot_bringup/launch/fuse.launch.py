# launch file for Fuse
#

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    default_fuse_params_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_bringup"),
        "params",
        "fuse_params.yaml"
    ])

    ld = LaunchDescription()
    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='fuse_params', default_value=default_fuse_params_path, description='Absolute path to [fuse] params file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='log_level', default_value='INFO', description='the Logging level'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='tf_timeout', default_value='40000000.0', description='the timeout waiting for a transform to be available'))

    fuse_node = launch_ros.actions.Node(
         package='fuse_optimizers',
         executable='fixed_lag_smoother_node',
         name='fuse_state_estimator_node',
         output='both',
         remappings=[
             ('odom','diffbot_base_controller/odom'),
             ('imu','imu_sensor_broadcaster/imu')
         ],
         arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
         parameters=[ 
             LaunchConfiguration('fuse_params'), 
             {'use_sim_time': LaunchConfiguration('use_sim_time')},
             {'tf_timeout': LaunchConfiguration('tf_timeout')},
             ]
    )
    ld.add_action(fuse_node)

    return ld
