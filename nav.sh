ros2 launch ros2_control_demo_example_2 diffbot.launch.py
ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=/dev/serial/by-path/platform-xhci-hcd.1-usbv2-0:1:1.0-port0
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=config/mapper_server_params_online_async.yaml
ros2 launch nav2_bringup navigation_launch.py params_file:=/home/alanros/ros2_diffbot_jazzy/src/example_2/bringup/config/nav2_params.yaml
ros2 run twist_mux twist_mux --ros-args --params-file ./src/example_2/bringup/config/twist_mux.yaml -r cmd_vel_out:=cmd_vel -p use_stamped:=true
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' publish_stamped_twist:=True joy_vel:=cmd_vel_joy