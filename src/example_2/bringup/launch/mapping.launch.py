from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ttyUSB1",
            description="Serial port for SLLIDAR",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam_params_file",
            default_value="./config/mymapper_params_online_async.yaml",
            description="SLAM Toolbox parameters file",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    lidar_port = LaunchConfiguration("lidar_port")
    slam_params_file = LaunchConfiguration("slam_params_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_demo_example_2"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_example_2"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_demo_description"), "diffbot/rviz", "diffbot.rviz"]
    )

    # Core nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffbot_base_controller",
            "--param-file",
            robot_controllers,
            "--controller-ros-args",
            "-r /diffbot_base_controller/cmd_vel:=/cmd_vel",
        ],
    )

    # Delay start events
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Include SLLIDAR launch
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sllidar_ros2"), "launch", "sllidar_a1_launch.py"]
            )
        ),
        launch_arguments={"serial_port": lidar_port}.items(),
    )

    # Include SLAM Toolbox launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
            )
        ),
        launch_arguments={"slam_params_file": slam_params_file}.items(),
    )

    # 延遲啟動 SLAM Toolbox (確保 TF 已準備好)
    slam_launch_delayed = TimerAction(
        period=5.0,  # 延遲 5 秒，可依需要調整
        actions=[slam_launch]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        sllidar_launch,
        slam_launch_delayed,  # 使用延遲啟動 SLAM Toolbox
        # delay_rviz_after_joint_state_broadcaster_spawner,  # 可選擇啟用
    ]

    return LaunchDescription(declared_arguments + nodes)
