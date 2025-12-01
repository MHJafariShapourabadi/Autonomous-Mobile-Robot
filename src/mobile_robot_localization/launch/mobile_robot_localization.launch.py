from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        name="use_python",
        default_value="False"
    )

    use_python = LaunchConfiguration(
        variable_name="use_python"
    )

    kalman_filter_py = Node(
        package="mobile_robot_localization",
        executable="kalman_filter.py",
        condition=IfCondition(use_python),
    )

    kalman_filter_cpp = Node(
        package="mobile_robot_localization",
        executable="kalman_filter",
        condition=UnlessCondition(use_python),
    )
    
    return LaunchDescription([
        use_python_arg,
        kalman_filter_py,
        kalman_filter_cpp
    ])