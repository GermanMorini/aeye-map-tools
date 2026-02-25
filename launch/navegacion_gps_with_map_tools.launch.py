#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    map_tools_share = get_package_share_directory("map_tools")
    navegacion_share = get_package_share_directory("navegacion_gps")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(navegacion_share, "config", "nav2_no_map_params.yaml"),
        description="Nav2 parameters file for navegacion_gps",
    )
    zones_file_arg = DeclareLaunchArgument(
        "zones_file",
        default_value="",
        description="Zone YAML file",
    )
    websocket_enabled_arg = DeclareLaunchArgument(
        "websocket_enabled",
        default_value="true",
        description="Enable websocket UI bridge",
    )
    websocket_host_arg = DeclareLaunchArgument(
        "websocket_host",
        default_value="0.0.0.0",
        description="Websocket host",
    )
    websocket_port_arg = DeclareLaunchArgument(
        "websocket_port",
        default_value="8765",
        description="Websocket port",
    )
    start_zone_server_arg = DeclareLaunchArgument(
        "start_zone_server",
        default_value="true",
        description="Start map_tools zone_server",
    )

    navegacion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navegacion_share, "launch", "navegacion.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    zone_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(map_tools_share, "launch", "zone_server.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_zone_server")),
        launch_arguments={
            "zones_file": LaunchConfiguration("zones_file"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "websocket_enabled": LaunchConfiguration("websocket_enabled"),
            "websocket_host": LaunchConfiguration("websocket_host"),
            "websocket_port": LaunchConfiguration("websocket_port"),
            "publish_nav2_mask": "true",
            "nav2_mask_topic": "/keepout_filter_mask",
            "nav2_filter_info_topic": "/costmap_filter_info",
            "nav2_filter_type": "0",
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            params_file_arg,
            zones_file_arg,
            websocket_enabled_arg,
            websocket_host_arg,
            websocket_port_arg,
            start_zone_server_arg,
            navegacion_launch,
            zone_server_launch,
        ]
    )
