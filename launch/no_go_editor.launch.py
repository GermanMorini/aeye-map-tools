from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    fromll_service = LaunchConfiguration("fromll_service")
    global_costmap_service = LaunchConfiguration("global_costmap_service")
    gps_topic = LaunchConfiguration("gps_topic")
    degrade_enabled = LaunchConfiguration("degrade_enabled")
    degrade_radius_m = LaunchConfiguration("degrade_radius_m")
    degrade_edge_cost = LaunchConfiguration("degrade_edge_cost")
    degrade_min_cost = LaunchConfiguration("degrade_min_cost")
    degrade_use_l2 = LaunchConfiguration("degrade_use_l2")
    use_fixed_mask_grid = LaunchConfiguration("use_fixed_mask_grid")
    mask_origin_x = LaunchConfiguration("mask_origin_x")
    mask_origin_y = LaunchConfiguration("mask_origin_y")
    mask_width = LaunchConfiguration("mask_width")
    mask_height = LaunchConfiguration("mask_height")
    mask_resolution = LaunchConfiguration("mask_resolution")
    snapshot_extent_m = LaunchConfiguration("snapshot_extent_m")
    snapshot_size_px = LaunchConfiguration("snapshot_size_px")
    snapshot_global_inset_px = LaunchConfiguration("snapshot_global_inset_px")
    snapshot_timeout_ms = LaunchConfiguration("snapshot_timeout_ms")
    plan_topic = LaunchConfiguration("plan_topic")
    cmd_vel_safe_topic = LaunchConfiguration("cmd_vel_safe_topic")
    nav_telemetry_hz = LaunchConfiguration("nav_telemetry_hz")
    manual_cmd_topic = LaunchConfiguration("manual_cmd_topic")
    manual_cmd_timeout_s = LaunchConfiguration("manual_cmd_timeout_s")
    manual_watchdog_hz = LaunchConfiguration("manual_watchdog_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("fromll_service", default_value="/fromLL"),
            DeclareLaunchArgument(
                "global_costmap_service", default_value="/global_costmap/get_costmap"
            ),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("degrade_enabled", default_value="true"),
            DeclareLaunchArgument("degrade_radius_m", default_value="2.0"),
            DeclareLaunchArgument("degrade_edge_cost", default_value="12"),
            DeclareLaunchArgument("degrade_min_cost", default_value="1"),
            DeclareLaunchArgument("degrade_use_l2", default_value="true"),
            DeclareLaunchArgument("use_fixed_mask_grid", default_value="true"),
            DeclareLaunchArgument("mask_origin_x", default_value="-150.0"),
            DeclareLaunchArgument("mask_origin_y", default_value="-150.0"),
            DeclareLaunchArgument("mask_width", default_value="3000"),
            DeclareLaunchArgument("mask_height", default_value="3000"),
            DeclareLaunchArgument("mask_resolution", default_value="0.1"),
            DeclareLaunchArgument("snapshot_extent_m", default_value="30.0"),
            DeclareLaunchArgument("snapshot_size_px", default_value="512"),
            DeclareLaunchArgument("snapshot_global_inset_px", default_value="160"),
            DeclareLaunchArgument("snapshot_timeout_ms", default_value="500"),
            DeclareLaunchArgument("plan_topic", default_value="/plan"),
            DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("nav_telemetry_hz", default_value="5.0"),
            DeclareLaunchArgument("manual_cmd_topic", default_value="/cmd_vel_safe"),
            DeclareLaunchArgument("manual_cmd_timeout_s", default_value="0.4"),
            DeclareLaunchArgument("manual_watchdog_hz", default_value="10.0"),
            Node(
                package="map_tools",
                executable="web_zone_server",
                name="web_zone_server",
                output="screen",
                parameters=[
                    {
                        "ws_host": ws_host,
                        "ws_port": ParameterValue(ws_port, value_type=int),
                        "fromll_service": fromll_service,
                        "global_costmap_service": global_costmap_service,
                        "mask_topic": "/keepout_filter_mask",
                        "filter_info_topic": "/costmap_filter_info",
                        "gps_topic": gps_topic,
                        "degrade_enabled": ParameterValue(degrade_enabled, value_type=bool),
                        "degrade_radius_m": ParameterValue(degrade_radius_m, value_type=float),
                        "degrade_edge_cost": ParameterValue(degrade_edge_cost, value_type=int),
                        "degrade_min_cost": ParameterValue(degrade_min_cost, value_type=int),
                        "degrade_use_l2": ParameterValue(degrade_use_l2, value_type=bool),
                        "use_fixed_mask_grid": ParameterValue(use_fixed_mask_grid, value_type=bool),
                        "mask_origin_x": ParameterValue(mask_origin_x, value_type=float),
                        "mask_origin_y": ParameterValue(mask_origin_y, value_type=float),
                        "mask_width": ParameterValue(mask_width, value_type=int),
                        "mask_height": ParameterValue(mask_height, value_type=int),
                        "mask_resolution": ParameterValue(mask_resolution, value_type=float),
                        "snapshot_extent_m": ParameterValue(snapshot_extent_m, value_type=float),
                        "snapshot_size_px": ParameterValue(snapshot_size_px, value_type=int),
                        "snapshot_global_inset_px": ParameterValue(
                            snapshot_global_inset_px, value_type=int
                        ),
                        "snapshot_timeout_ms": ParameterValue(
                            snapshot_timeout_ms, value_type=int
                        ),
                        "plan_topic": plan_topic,
                        "cmd_vel_safe_topic": cmd_vel_safe_topic,
                        "nav_telemetry_hz": ParameterValue(
                            nav_telemetry_hz, value_type=float
                        ),
                        "manual_cmd_topic": manual_cmd_topic,
                        "manual_cmd_timeout_s": ParameterValue(
                            manual_cmd_timeout_s, value_type=float
                        ),
                        "manual_watchdog_hz": ParameterValue(
                            manual_watchdog_hz, value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
