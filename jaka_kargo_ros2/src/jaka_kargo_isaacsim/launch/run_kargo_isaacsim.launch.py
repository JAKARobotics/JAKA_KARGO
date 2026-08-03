import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _start_isaac_sim(context):
    isaac_root = Path(
        LaunchConfiguration("isaac_root").perform(context)
    ).expanduser()

    usd_package = LaunchConfiguration("usd_pkg").perform(context)
    usd_relative_path = LaunchConfiguration("usd_rel").perform(context)

    isaac_python = isaac_root / "python.sh"
    isaac_ros_lib = (
        isaac_root
        / "exts"
        / "isaacsim.ros2.bridge"
        / "humble"
        / "lib"
    )

    isaac_script = (
        Path(get_package_share_directory("jaka_kargo_isaacsim"))
        / "scripts"
        / "isaacsim_moveit.py"
    )

    usd_path = (
        Path(get_package_share_directory(usd_package))
        / usd_relative_path
    )

    if not isaac_python.is_file():
        raise RuntimeError(
            f"Isaac Sim Python launcher not found: {isaac_python}"
        )

    if not isaac_ros_lib.is_dir():
        raise RuntimeError(
            f"Isaac ROS 2 library directory not found: {isaac_ros_lib}"
        )

    if not isaac_script.is_file():
        raise RuntimeError(
            f"Isaac integration script not found: {isaac_script}"
        )

    if not usd_path.is_file():
        raise RuntimeError(
            f"KARGO USD file not found: {usd_path}"
        )

    # Begin with the launch environment, but remove paths that would make
    # Isaac Sim load /opt/ros/humble libraries inside its own process.
    isaac_environment = os.environ.copy()

    for variable in (
        "PYTHONPATH",
        "PYTHONHOME",
        "AMENT_PREFIX_PATH",
        "COLCON_PREFIX_PATH",
        "ROS_VERSION",
        "ROS_PYTHON_VERSION",
        "VIRTUAL_ENV",
        "CONDA_PREFIX",
    ):
        isaac_environment.pop(variable, None)

    isaac_environment.update(
        {
            "ROS_DISTRO": "humble",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "LD_LIBRARY_PATH": str(isaac_ros_lib),
            "KARGO_USD_PATH": str(usd_path),
        }
    )

    # ROS_DOMAIN_ID and FASTRTPS_DEFAULT_PROFILES_FILE are intentionally
    # retained from the calling terminal.
    return [
        ExecuteProcess(
            cmd=[
                str(isaac_python),
                str(isaac_script),
            ],
            env=isaac_environment,
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "isaac_root",
                default_value=str(Path.home() / "isaacsim"),
                description="Isaac Sim installation directory",
            ),
            DeclareLaunchArgument(
                "usd_pkg",
                default_value="jaka_kargo_description",
            ),
            DeclareLaunchArgument(
                "usd_rel",
                default_value=(
                    "urdf/jaka_kargo/"
                    "jaka_kargo_moveit.usd"
                ),
            ),
            OpaqueFunction(function=_start_isaac_sim),
        ]
    )