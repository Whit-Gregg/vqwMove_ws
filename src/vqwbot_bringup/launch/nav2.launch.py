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

    ekf_config_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_bringup'),
        'params',
        'ekf.yaml'
    ])
    
    # xacro_urdf_file_path = PathJoinSubstitution([
    #     FindPackageShare('vqwbot_bringup'),
    #     'urdf',
    #     'vqwbot.urdf.xacro'
    # ])

    # default_rviz_config_path = PathJoinSubstitution([
    #     FindPackageShare("vqwbot_bringup"),
    #     "params",
    #     "urdf_config.rviz"
    # ])


    # robot_description =  {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    ld = LaunchDescription()
    # ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_urdf_file_path, description='Absolute path to robot urdf xarco file'))
    # ld.add_action(launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time'))


    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[ ekf_config_path , {'use_sim_time': LaunchConfiguration('use_sim_time')} ]
    )
    ld.add_action(robot_localization_node)


    # slam_toolbox_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #     [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')])
    # )
    # ld.add_action(slam_toolbox_launch)

    navigation_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')])
    )
    ld.add_action(navigation_launch)



    return ld
