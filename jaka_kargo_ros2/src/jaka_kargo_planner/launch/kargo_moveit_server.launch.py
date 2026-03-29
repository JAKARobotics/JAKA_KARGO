import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare 'ip' and 'model' arguments
        DeclareLaunchArgument('ip', default_value='127.0.0.1', description='IP address'),

        # Launch the 'kargo_moveit_server' node from the 'jaka_kargo_planner' package
        Node(
            package='jaka_kargo_planner',
            executable='kargo_moveit_server',  # the executable to run
            name='kargo_moveit_server',
            output='screen',
            # arguments=["--ros-args", "--log-level", "kargo_moveit_server:=debug"],
            parameters=[
                {'ip': LaunchConfiguration('ip')},  # Pass 'ip' parameter
            ],
        ),
    ])