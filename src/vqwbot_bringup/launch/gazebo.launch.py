import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_bringup'),
        'params',
        'vqwbot_world.sdf'
    ])
    
    default_rviz_config_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_bringup'),
        'params',
        'vqwbot_sim.rviz'
    ])
    
    ekf_config_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_bringup'),
        'params',
        'ekf.yaml'
    ])
    
    xacro_urdf_file_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_bringup'),
        'urdf',
        'vqwbot_sim.urdf.xacro'
    ])

    robot_description =  {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    # robot_controllers_path = PathJoinSubstitution([
    #     FindPackageShare("vqwbot_bringup"),
    #     "params",
    #     "roboclaw_controllers.yaml"
    # ])

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, robot_controllers_path],
    #     output="both",
    # )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[LaunchConfiguration('model')] #Add this line
        # parameters=[robot_description], #Remove this line
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[],
        output="both",
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'vqwbot', '-topic', 'robot_description'],
        output='screen'
    )

    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[ ekf_config_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

#         launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),


    nodes = [
        launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_urdf_file_path, description='Absolute path to robot urdf xarco file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),     
        launch.actions.ExecuteProcess(cmd=['gz', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node
    ]

    return LaunchDescription(nodes)
