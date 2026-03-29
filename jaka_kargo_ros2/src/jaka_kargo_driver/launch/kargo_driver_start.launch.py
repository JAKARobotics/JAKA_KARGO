import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'ip' argument
        DeclareLaunchArgument('ip', default_value='127.0.0.1', description='IP address'),

        # Print the IP to the log for debugging
        LogInfo(
            msg=["The IP address is: ", LaunchConfiguration('ip')]  # Correct substitution usage
        ),

        # Launch the 'jaka_kargo_driver' node with the provided 'ip' parameter
        Node(
            package='jaka_kargo_driver',
            executable='jaka_kargo_driver',  # the executable to run
            name='jaka_kargo_driver',
            output='screen',
            parameters=[{'ip': LaunchConfiguration('ip')}],  # pass the 'ip' parameter
        ),
    ])
