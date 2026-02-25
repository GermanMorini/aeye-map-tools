#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_dir = os.path.join(get_package_share_directory("map_tools"), "config")
    default_zones_file = os.path.join(config_dir, "zones.web.yaml")

    zones_file_arg = DeclareLaunchArgument(
        "zones_file",
        default_value=default_zones_file,
        description="Path to the zone YAML file",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="map",
        description="Frame id used for zone coordinates",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    marker_topic_arg = DeclareLaunchArgument(
        "marker_topic",
        default_value="/zone_markers",
        description="MarkerArray output topic",
    )
    state_topic_arg = DeclareLaunchArgument(
        "state_topic",
        default_value="/zones_state",
        description="Serialized zone state output topic",
    )
    set_topic_arg = DeclareLaunchArgument(
        "set_topic",
        default_value="/zones_cmd",
        description="JSON zone command input topic",
    )
    load_on_start_arg = DeclareLaunchArgument(
        "load_on_start",
        default_value="true",
        description="Load zone file when node starts",
    )
    websocket_enabled_arg = DeclareLaunchArgument(
        "websocket_enabled",
        default_value="true",
        description="Enable websocket server for remote UI",
    )
    websocket_host_arg = DeclareLaunchArgument(
        "websocket_host",
        default_value="0.0.0.0",
        description="Websocket listen host",
    )
    websocket_port_arg = DeclareLaunchArgument(
        "websocket_port",
        default_value="8765",
        description="Websocket listen port",
    )
    projection_enabled_arg = DeclareLaunchArgument(
        "projection_enabled",
        default_value="true",
        description="Enable fromLL/toLL projection bridge",
    )
    from_ll_service_arg = DeclareLaunchArgument(
        "from_ll_service",
        default_value="/fromLL",
        description="robot_localization fromLL service name",
    )
    to_ll_service_arg = DeclareLaunchArgument(
        "to_ll_service",
        default_value="/toLL",
        description="robot_localization toLL service name",
    )
    projection_timeout_arg = DeclareLaunchArgument(
        "projection_request_timeout_sec",
        default_value="10.0",
        description="Timeout for projection requests over websocket",
    )
    publish_nav2_mask_arg = DeclareLaunchArgument(
        "publish_nav2_mask",
        default_value="true",
        description="Publish keepout OccupancyGrid and CostmapFilterInfo",
    )
    nav2_mask_topic_arg = DeclareLaunchArgument(
        "nav2_mask_topic",
        default_value="/keepout_filter_mask",
        description="OccupancyGrid keepout mask topic",
    )
    nav2_filter_info_topic_arg = DeclareLaunchArgument(
        "nav2_filter_info_topic",
        default_value="/costmap_filter_info",
        description="CostmapFilterInfo topic",
    )
    nav2_filter_type_arg = DeclareLaunchArgument(
        "nav2_filter_type",
        default_value="0",
        description="Costmap filter type (0=keepout)",
    )
    nav2_mask_base_arg = DeclareLaunchArgument(
        "nav2_mask_base",
        default_value="0.0",
        description="CostmapFilterInfo base value",
    )
    nav2_mask_multiplier_arg = DeclareLaunchArgument(
        "nav2_mask_multiplier",
        default_value="1.0",
        description="CostmapFilterInfo multiplier",
    )
    mask_resolution_arg = DeclareLaunchArgument(
        "mask_resolution",
        default_value="0.1",
        description="Mask resolution in meters/cell",
    )
    mask_width_arg = DeclareLaunchArgument(
        "mask_width",
        default_value="400",
        description="Mask width in cells",
    )
    mask_height_arg = DeclareLaunchArgument(
        "mask_height",
        default_value="400",
        description="Mask height in cells",
    )
    mask_origin_x_arg = DeclareLaunchArgument(
        "mask_origin_x",
        default_value="-20.0",
        description="Mask origin X in map frame",
    )
    mask_origin_y_arg = DeclareLaunchArgument(
        "mask_origin_y",
        default_value="-20.0",
        description="Mask origin Y in map frame",
    )
    mask_default_value_arg = DeclareLaunchArgument(
        "mask_default_value",
        default_value="0",
        description="Mask default occupancy value",
    )
    mask_no_go_value_arg = DeclareLaunchArgument(
        "mask_no_go_value",
        default_value="100",
        description="Mask value used for no-go zones",
    )
    mask_auto_resize_arg = DeclareLaunchArgument(
        "mask_auto_resize",
        default_value="true",
        description="Automatically resize mask extents to include all active no-go polygons",
    )
    mask_auto_resize_margin_arg = DeclareLaunchArgument(
        "mask_auto_resize_margin_m",
        default_value="2.0",
        description="Margin around polygons when auto-resizing mask (meters)",
    )

    node = Node(
        package="map_tools",
        executable="zone_server",
        name="zone_server",
        output="screen",
        parameters=[
            {
                "zones_file": LaunchConfiguration("zones_file"),
                "frame_id": LaunchConfiguration("frame_id"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "state_topic": LaunchConfiguration("state_topic"),
                "set_topic": LaunchConfiguration("set_topic"),
                "load_on_start": LaunchConfiguration("load_on_start"),
                "websocket_enabled": LaunchConfiguration("websocket_enabled"),
                "websocket_host": LaunchConfiguration("websocket_host"),
                "websocket_port": LaunchConfiguration("websocket_port"),
                "projection_enabled": LaunchConfiguration("projection_enabled"),
                "from_ll_service": LaunchConfiguration("from_ll_service"),
                "to_ll_service": LaunchConfiguration("to_ll_service"),
                "projection_request_timeout_sec": LaunchConfiguration("projection_request_timeout_sec"),
                "publish_nav2_mask": LaunchConfiguration("publish_nav2_mask"),
                "nav2_mask_topic": LaunchConfiguration("nav2_mask_topic"),
                "nav2_filter_info_topic": LaunchConfiguration("nav2_filter_info_topic"),
                "nav2_filter_type": LaunchConfiguration("nav2_filter_type"),
                "nav2_mask_base": LaunchConfiguration("nav2_mask_base"),
                "nav2_mask_multiplier": LaunchConfiguration("nav2_mask_multiplier"),
                "mask_resolution": LaunchConfiguration("mask_resolution"),
                "mask_width": LaunchConfiguration("mask_width"),
                "mask_height": LaunchConfiguration("mask_height"),
                "mask_origin_x": LaunchConfiguration("mask_origin_x"),
                "mask_origin_y": LaunchConfiguration("mask_origin_y"),
                "mask_default_value": LaunchConfiguration("mask_default_value"),
                "mask_no_go_value": LaunchConfiguration("mask_no_go_value"),
                "mask_auto_resize": LaunchConfiguration("mask_auto_resize"),
                "mask_auto_resize_margin_m": LaunchConfiguration("mask_auto_resize_margin_m"),
            }
        ],
    )

    return LaunchDescription(
        [
            zones_file_arg,
            frame_id_arg,
            use_sim_time_arg,
            marker_topic_arg,
            state_topic_arg,
            set_topic_arg,
            load_on_start_arg,
            websocket_enabled_arg,
            websocket_host_arg,
            websocket_port_arg,
            projection_enabled_arg,
            from_ll_service_arg,
            to_ll_service_arg,
            projection_timeout_arg,
            publish_nav2_mask_arg,
            nav2_mask_topic_arg,
            nav2_filter_info_topic_arg,
            nav2_filter_type_arg,
            nav2_mask_base_arg,
            nav2_mask_multiplier_arg,
            mask_resolution_arg,
            mask_width_arg,
            mask_height_arg,
            mask_origin_x_arg,
            mask_origin_y_arg,
            mask_default_value_arg,
            mask_no_go_value_arg,
            mask_auto_resize_arg,
            mask_auto_resize_margin_arg,
            node,
        ]
    )
