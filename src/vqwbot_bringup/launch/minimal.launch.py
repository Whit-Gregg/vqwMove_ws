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
        'vqwbot.urdf.xacro'
    ])

    robot_description =  {'robot_description': 
                            launch_ros.parameter_descriptions.ParameterValue(
                                Command(['xacro ', LaunchConfiguration('model')])
                                , value_type=str)
                                }

    robot_controllers_path = PathJoinSubstitution([
        FindPackageShare("vqwbot_bringup"),
        "params",
        "combined_controllers.yaml"
    ])

    remappings = [
                    ('/diffbot_base_controller/cmd_vel', '/cmd_vel'),
                  ]

    ld = LaunchDescription()
    ld.add_action(launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time'))
    ld.add_action(launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_urdf_file_path, description='Absolute path to robot urdf xarco file'))


    # PUBLISH ROBOT DESCRIPTION, and TF Transforms for Joints
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        # remappings=[],
        output="both",
    )
    ld.add_action(robot_state_pub_node)
 

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #arguments=["--ros-args", "--log-level", "DEBUG"],
        #parameters=[robot_description, robot_controllers_path],
        parameters=[robot_controllers_path],
        remappings=remappings,
        output="both",
    )
    ld.add_action(ros2_control_node)


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )
    ld.add_action(joint_state_broadcaster_spawner)


    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", 
                   "--controller-manager", "/controller_manager",
                   ],
        output="both",
    )
    ld.add_action(imu_sensor_broadcaster_spawner)


    diffbot_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="DiffDriveController",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )
    ld.add_action(diffbot_base_controller_spawner)
    
    #--------- position_controller ----------
    # ---------
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="CamServoController",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )
    ld.add_action(position_controller_spawner)

    return ld


