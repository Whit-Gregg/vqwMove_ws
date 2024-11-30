import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # xacro_urdf_file_path = PathJoinSubstitution([
    #     FindPackageShare('vqwbot_bringup'),
    #     'urdf',
    #     'vqwbot.urdf.xacro'
    # ])

    default_rviz2_config_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_bringup"),
        "params",
        "urdf_config.rviz"
    ])

    # robot_description =  {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )



    nodes = [
        # launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_urdf_file_path,
        #                                     description='Absolute path to robot urdf xarco file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz2_config_path,
                                            description='Absolute path to rviz config file'),

        joint_state_publisher_gui_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)
