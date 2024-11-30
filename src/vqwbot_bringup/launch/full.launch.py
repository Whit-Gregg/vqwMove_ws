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

    # nav2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'nav2.launch.py')])
    # )
    # ld.add_action(nav2_launch)

    # rviz_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     [os.path.join(get_package_share_directory('vqwbot_bringup'), 'launch', 'rviz.launch.py')])
    # )
    # ld.add_action(rviz_launch)


    return ld
