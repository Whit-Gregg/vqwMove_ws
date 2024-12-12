import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    xacro_urdf_file_path = PathJoinSubstitution([
        FindPackageShare('vqwbot_bringup'),
        'urdf',
        'vqwbot_bno08x.urdf.xacro'
    ])

    robot_description =  {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    robot_controllers_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_bringup"),
        "params",
        "bno08x_controllers.yaml"
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        arguments=["--ros-args", "--log-level", "DEBUG"],
        parameters=[robot_description, robot_controllers_path],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        remappings=[],
        output="both",
    )


    # imu_sensor_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["--controller-manager", "/controller_manager",
    #                "--controller-manager-timeout","25",
    #                "--activate",
    #                "imu_sensor_broadcaster"],
    #     output="both",
    # )


    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", 
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "25",
                   ],
        output="both",
    )

    # diffbot_base_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    #     output="both",
    # )



    nodes = [
        launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_urdf_file_path, description='Absolute path to robot urdf xarco file'),
        ros2_control_node,
        robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # imu_sensor_broadcaster_spawner,
        # diffbot_base_controller_spawner,
    ]

    return LaunchDescription(nodes)
