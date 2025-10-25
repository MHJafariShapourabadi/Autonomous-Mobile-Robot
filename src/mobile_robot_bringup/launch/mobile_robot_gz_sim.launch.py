from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ros_distro = os.environ["ROS_DISTRO"]
    is_gz = "true" if ros_distro == "jazzy" else "false"

    urdf_path = DeclareLaunchArgument(
        name="urdf_path",
        default_value=PathJoinSubstitution([FindPackageShare("mobile_robot_description"), "urdf/mobile_robot.urdf.xacro"])
    )

    world_path = DeclareLaunchArgument(
        name="world_path",
        default_value=PathJoinSubstitution([FindPackageShare("mobile_robot_description"), "worlds/Depot/model.sdf -r"])
    )

    rviz_config_path = DeclareLaunchArgument(
        name="rviz_config_path",
        default_value=PathJoinSubstitution([FindPackageShare("mobile_robot_description"), "rviz/default_config.rviz"])
    )

    if ros_distro == "jazzy":
        bridge_config_path = DeclareLaunchArgument(
            name="bridge_config_path",
            default_value=PathJoinSubstitution([FindPackageShare("mobile_robot_bringup"), "config/gz_bridge.yaml"])
        )
    else:
        bridge_config_path = DeclareLaunchArgument(
            name="bridge_config_path",
            default_value=PathJoinSubstitution([FindPackageShare("mobile_robot_bringup"), "config/ign_bridge.yaml"])
        )
    
    robot_description = Command([
        "xacro ",
        LaunchConfiguration("urdf_path"),
        " is_gz:=",
        is_gz,
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description
        }]
    )

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

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch/gz_sim.launch.py"
            ])
        ),
        launch_arguments=[("gz_args", LaunchConfiguration("world_path"))]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"]
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": LaunchConfiguration("bridge_config_path")
        }]
    )

    lidar_frame_id_converter = Node(
        package="mobile_robot_bringup",
        executable="pointcloud_frame_id_converter",
        name="pointcloud_frame_id_converter_lidar",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "lidar_link",
            "subscription_topic": "/lidar/pointcloud/points",
            "publisher_topic": "/lidar/pointcloud/points/corrected",
        }]
    )

    depth_camera_frame_id_converter = Node(
        package="mobile_robot_bringup",
        executable="pointcloud_frame_id_converter",
        name="pointcloud_frame_id_converter_depth_camera",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "zed_camera_link",
            "subscription_topic": "/depth_camera/points",
            "publisher_topic": "/depth_camera/points/corrected",
        }]
    )

    mobile_robot_controller = TimerAction(
        period=15.0,  # Wait 15 seconds for Gazebo + robot to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("mobile_robot_controller"),
                        "launch/controller.launch.py"
                    ])
                )
            )
        ]
    )

    rviz = Node(
        executable="rviz2",
        package="rviz2",
        output="screen",
        arguments=[
            "-d", LaunchConfiguration("rviz_config_path"),
        ]
    )

    start_rviz = TimerAction(
        period=5.0, # wait 20 seconds
        actions=[
            rviz,
        ]
    )

    # # Absolute path to your venv python
    # venv_python = "/home/fasta/PythonVEnvs/rosailab/bin/python"

    # # Start your object detector via `python -m ...`
    # object_detector = ExecuteProcess(
    #     cmd=[venv_python, "-u", "-m", "mobile_robot_object_detector.object_detector", "--ros-args"],
    #     name="object_detector",
    #     output="screen"
    #     # (inherits the environment from the launch process so your sourced workspace
    #     #  PYTHONPATH/AMENT_PREFIX_PATH are kept)
    # )

    # # Optional: wait a few seconds so the bridge/camera topic is alive first
    # start_object_detector = TimerAction(period=3.0, actions=[object_detector])

    return LaunchDescription([
        urdf_path,
        world_path,
        rviz_config_path,
        bridge_config_path,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        ros_gz_bridge,
        lidar_frame_id_converter,
        depth_camera_frame_id_converter,
        mobile_robot_controller,
        # start_object_detector,
        start_rviz
    ])
