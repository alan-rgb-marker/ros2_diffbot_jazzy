from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

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


    return LaunchDescription([
        diffbot_launch,
        nav2_launch
    ])