from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    diffbot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_2"),
                    "launch",
                    "diffbot.launch.py",
                ]
            )
        ),
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py",
                ]
            ),
        ),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [
                    FindPackageShare("ros2_control_demo_example_2"),
                    "config",
                    "nav2_params.yaml",
                ]
            )
        }.items(),
    )
    
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        # name="map_server",
        output="both",
        parameters=[
            {
                'yaml_filename': "/home/alanros/room.yaml",
                'topic_name': "map",
                'frame_id': "map",
                # 'introspection_mode': 'enabled',
            }
        ],
    )
    

    delayed_map_server = TimerAction(
        period=5.0,
        actions=[map_server],
    )
    
    
    # delayed_nav2 = TimerAction(
    #     period=7.0,
    #     actions=[nav2_launch],
    # )
    
    delayed_nav2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=map_server,
            on_exit=[nav2_launch],
        )
    )

    return LaunchDescription([
        diffbot_launch,
        delayed_map_server,
        delayed_nav2,
    ])