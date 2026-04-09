from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    processes_file = LaunchConfiguration("processes_file")
    process_file_logging = LaunchConfiguration("process_file_logging")

    controller_launch = (
        Path(get_package_share_directory("controller_server"))
        / "launch"
        / "controller_server.launch.py"
    )
    controller_sim_launch = (
        Path(get_package_share_directory("controller_server"))
        / "launch"
        / "controller_server_sim.launch.py"
    )
    default_processes_file = (
        Path(get_package_share_directory("utilities")) / "config" / "process_list.json"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim", default_value="false"),
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("processes_file", default_value=str(default_processes_file)),
            DeclareLaunchArgument("process_file_logging", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(controller_launch)),
                condition=UnlessCondition(use_sim),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(controller_sim_launch)),
                condition=IfCondition(use_sim),
            ),
            Node(
                package="map_tools",
                executable="web_zone_server",
                name="web_zone_server",
                output="screen",
                parameters=[
                    {
                        "ws_host": ws_host,
                        "ws_port": ws_port,
                    }
                ],
            ),
            Node(
                package="utilities",
                executable="process_executor",
                name="process_executor",
                output="screen",
                parameters=[
                    {
                        "processes_file": processes_file,
                        "file_logging": process_file_logging,
                    }
                ],
            ),
        ]
    )
