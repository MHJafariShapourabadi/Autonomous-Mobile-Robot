from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = os.path.join(get_package_share_directory('mobile_robot_description'),'urdf/mobile_robot.urdf.xacro') 

    rviz_config_path = os.path.join(get_package_share_directory('mobile_robot_description'), 'rviz/default_config.rviz')
    
    robot_state_publisher= Node(
        executable="robot_state_publisher",
        package="robot_state_publisher",
        parameters=[
            {
                'robot_description': Command(["xacro ", urdf_path])
            }
        ]
    )

    joint_state_publisher = Node(
        executable="joint_state_publisher_gui",
        package="joint_state_publisher_gui",
    )

    rviz = Node(
        executable="rviz2",
        package="rviz2",
        arguments=[
            "-d", rviz_config_path,
        ]
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(rviz)

    return ld