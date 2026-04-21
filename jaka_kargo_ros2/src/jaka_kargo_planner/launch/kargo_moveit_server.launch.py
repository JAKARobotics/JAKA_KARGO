import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Robot connection
        DeclareLaunchArgument('ip', default_value='127.0.0.1', description='IP address'),
        # External axis parameters
        DeclareLaunchArgument('ext_vel', default_value='50.0', description='External axis velocity'),
        DeclareLaunchArgument('ext_acc', default_value='50.0', description='External axis acceleration'),
        # AGV namespace / map
        DeclareLaunchArgument('agv_ns', default_value='/JAGV_O_01', description='AGV namespace'),
        DeclareLaunchArgument('agv_map_name', default_value='testmap', description='AGV map name'),
        # AGV motion parameters
        DeclareLaunchArgument('agv_linear_speed', default_value='0.5', description='AGV linear speed'),
        DeclareLaunchArgument('agv_angular_speed', default_value='0.2', description='AGV angular speed'),
        DeclareLaunchArgument('agv_dece_distance', default_value='1.0', description='AGV deceleration distance'),
        DeclareLaunchArgument('agv_stop_distance', default_value='0.2', description='AGV stop distance'),
        
        # Launch the 'kargo_moveit_server' node from the 'jaka_kargo_planner' package
        Node(
            package='jaka_kargo_planner',
            executable='kargo_moveit_server',  # the executable to run
            name='kargo_moveit_server',
            output='screen',
            # arguments=["--ros-args", "--log-level", "kargo_moveit_server:=debug"],
            parameters=[
                {'ip': LaunchConfiguration('ip')},  # Pass 'ip' parameter
                {'ext_vel': LaunchConfiguration('ext_vel')},
                {'ext_acc': LaunchConfiguration('ext_acc')},
                {'agv_ns': LaunchConfiguration('agv_ns')},
                {'agv_map_name': LaunchConfiguration('agv_map_name')},
                {'agv_linear_speed': LaunchConfiguration('agv_linear_speed')},
                {'agv_angular_speed': LaunchConfiguration('agv_angular_speed')},
                {'agv_dece_distance': LaunchConfiguration('agv_dece_distance')},
                {'agv_stop_distance': LaunchConfiguration('agv_stop_distance')},
            ],
        ),
    ])