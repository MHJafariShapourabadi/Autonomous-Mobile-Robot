from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        name="use_python",
        default_value="False"
    )

    use_python = LaunchConfiguration(
        variable_name="use_python"
    )

    use_custom_sensor_fusion_arg = DeclareLaunchArgument(
        name="use_custom_sensor_fusion",
        default_value="False"
    )

    use_custom_sensor_fusion = LaunchConfiguration(
        variable_name="use_custom_sensor_fusion"
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

    custom_sensor_fusion = GroupAction(
        actions=[
            kalman_filter_py,
            kalman_filter_cpp
        ],
        condition=IfCondition(use_custom_sensor_fusion)
    )

    ekf_static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"]
    )

    ekf_imu_frame_id_converter = Node(
        package="mobile_robot_bringup",
        executable="imu_frame_id_converter",
        name="imu_frame_id_converter",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "base_footprint_ekf",
            "subscription_topic": "imu/data_raw",
            "publisher_topic": "imu/ekf",
        }]
    )

    ekf_config = PathJoinSubstitution([
        FindPackageShare("mobile_robot_localization"),
        "config",
        "ekf_config.yaml"
    ])

    ekf_filter = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config]
    )

    ekf_sensor_fusion = GroupAction(
        actions=[
            ekf_static_transform_publisher,
            ekf_imu_frame_id_converter,
            ekf_filter,            
        ],
        condition=UnlessCondition(use_custom_sensor_fusion)
    )
    
    return LaunchDescription([
        use_python_arg,
        use_custom_sensor_fusion_arg,
        custom_sensor_fusion,
        ekf_sensor_fusion,
    ])