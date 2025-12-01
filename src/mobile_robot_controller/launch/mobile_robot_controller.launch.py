from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    # context = LaunchContext()

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

    use_simple_controller_arg = DeclareLaunchArgument(
        name="use_simple_controller",
        default_value="False"
    )

    use_simple_controller = LaunchConfiguration("use_simple_controller")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "30.0",
        ]
    )

    diff_drive_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "30.0",
            "--controller-ros-args", "-r /diff_drive_base_controller/cmd_vel:=/cmd_vel", 
            "--controller-ros-args", "-r /diff_drive_base_controller/odom:=/odom_perfect",
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    use_python_arg = DeclareLaunchArgument(
        name="use_python",
        default_value="False",
    )

    wheel_radius_arg = DeclareLaunchArgument(
        name="wheel_radius",
        default_value="0.1",
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        name="wheel_radius_error",
        default_value="0.005",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        name="wheel_separation",
        default_value="0.35",
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        name="wheel_separation_error",
        default_value="0.02",
    )

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_radius_error = LaunchConfiguration("wheel_radius_error")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_separation_error = LaunchConfiguration("wheel_separation_error")

    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "30.0",
        ]
    )

    cmd_vel_to_wheel_vel_py = Node(
        package="mobile_robot_controller",
        executable="simple_controller.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            'use_sim_time': True,
        }],
        condition=IfCondition(use_python),
    )

    cmd_vel_to_wheel_vel_cpp = Node(
        package="mobile_robot_controller",
        executable="simple_controller",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_radius_error": wheel_radius_error,
            'use_sim_time': True,
        }],
        condition=UnlessCondition(use_python),
    )

    perfect_odom_py = Node(
        package="mobile_robot_controller",
        executable="perfect_odom.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            'use_sim_time': True,
        }],
        condition=IfCondition(use_python),
    )

    noisy_odom_py = Node(
        package="mobile_robot_controller",
        executable="noisy_odom.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "wheel_radius_error": wheel_radius_error,
            "wheel_separation_error": wheel_separation_error,
            'use_sim_time': True,
        }],
        condition=IfCondition(use_python),
    )

    perfect_odom_cpp = Node(
        package="mobile_robot_controller",
        executable="perfect_odom",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            'use_sim_time': True,
        }],
        condition=UnlessCondition(use_python),
    )

    noisy_odom_cpp = Node(
        package="mobile_robot_controller",
        executable="noisy_odom",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
            "wheel_radius_error": wheel_radius_error,
            "wheel_separation_error": wheel_separation_error,
            'use_sim_time': True,
        }],
        condition=UnlessCondition(use_python),
    )

    simple_velocity_controller = GroupAction(
        actions=[
            simple_velocity_controller_spawner,
            cmd_vel_to_wheel_vel_py,
            cmd_vel_to_wheel_vel_cpp,
            perfect_odom_py,
            perfect_odom_cpp,
        ],
        condition=IfCondition(use_simple_controller)
    )

    controller_spawning = TimerAction(
    period=10.0,
    actions=[
        joint_state_broadcaster_spawner,
        simple_velocity_controller,
        diff_drive_base_controller_spawner,
        noisy_odom_py,
        noisy_odom_cpp,
    ])

    return LaunchDescription([
        use_simple_controller_arg,
        use_python_arg,
        wheel_radius_arg,
        wheel_radius_error_arg,
        wheel_separation_arg,
        wheel_separation_error_arg,
        controller_spawning,
    ])