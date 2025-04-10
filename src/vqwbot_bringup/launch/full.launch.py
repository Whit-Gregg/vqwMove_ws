from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    minimal_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'minimal.launch.py')])
    )
    ld.add_action(minimal_launch)

    sllidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_s2_launch.py')])
    )
    ld.add_action(sllidar_launch)

    joy_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'joy.launch.py')])
    )
    ld.add_action(joy_launch)

    # joy_pan_tilt_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'joy_pan_tilt.launch.py')])
    # )
    # ld.add_action(joy_pan_tilt_launch)


    fuse_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'fuse.launch.py')])
    )
    ld.add_action(fuse_launch)
    
    
    slam_toolbox_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'slam_toolbox.launch.py')])
    )
    ld.add_action(slam_toolbox_launch)

    return ld
