from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # urdf_path = DeclareLaunchArgument(
    #     name="urdf_path",
    #     default_value=PathJoinSubstitution([FindPackageShare("mobile_robot_description"), "urdf/mobile_robot.urdf.xacro"])
    # )

    # robot_description = Command(["xacro ", LaunchConfiguration("urdf_path")])

    # Launch the controller manager node with the robot description for real harware control (not simulation)
    # controller_manager = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     name='controller_manager',
    #     parameters=[
    #         {'robot_description': robot_description},  # This is the key line!
    #         {'use_sim_time': False},  # Note: False for real hardware
    #         PathJoinSubstitution([
    #             FindPackageShare("mobile_robot_controller"),
    #             "config/mobile_robot_controllers.yaml"
    #         ]),
    #     ],
    #     output='screen',
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "30.0",
        ]
    )

    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "20.0",
        ]
    )

    # diff_drive_base_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "diff_drive_base_controller",
    #         "--controller-manager", "/controller_manager",
    #         "--switch-timeout", "30.0",
    #     ]
    # )

    controller_spawning = TimerAction(
    period=10.0,
    actions=[
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        # diff_drive_base_controller_spawner,
    ]
)

    return LaunchDescription([
        controller_spawning,
    ])