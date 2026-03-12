from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    gps_topic = LaunchConfiguration("gps_topic")
    map_frame = LaunchConfiguration("map_frame")

    zones_set_geojson_service = LaunchConfiguration("zones_set_geojson_service")
    zones_get_state_service = LaunchConfiguration("zones_get_state_service")
    zones_reload_service = LaunchConfiguration("zones_reload_service")

    nav_set_goal_service = LaunchConfiguration("nav_set_goal_service")
    nav_cancel_goal_service = LaunchConfiguration("nav_cancel_goal_service")
    nav_brake_service = LaunchConfiguration("nav_brake_service")
    nav_set_manual_mode_service = LaunchConfiguration("nav_set_manual_mode_service")
    nav_get_state_service = LaunchConfiguration("nav_get_state_service")
    teleop_cmd_topic = LaunchConfiguration("teleop_cmd_topic")

    nav_snapshot_service = LaunchConfiguration("nav_snapshot_service")
    nav_telemetry_topic = LaunchConfiguration("nav_telemetry_topic")
    camera_pan_service = LaunchConfiguration("camera_pan_service")
    camera_status_service = LaunchConfiguration("camera_status_service")

    request_timeout_s = LaunchConfiguration("request_timeout_s")
    snapshot_request_timeout_s = LaunchConfiguration("snapshot_request_timeout_s")
    set_zones_timeout_s = LaunchConfiguration("set_zones_timeout_s")
    set_goal_timeout_s = LaunchConfiguration("set_goal_timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8766"),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument(
                "zones_set_geojson_service", default_value="/zones_manager/set_geojson"
            ),
            DeclareLaunchArgument(
                "zones_get_state_service", default_value="/zones_manager/get_state"
            ),
            DeclareLaunchArgument(
                "zones_reload_service", default_value="/zones_manager/reload_from_disk"
            ),
            DeclareLaunchArgument(
                "nav_set_goal_service", default_value="/nav_command_server/set_goal_ll"
            ),
            DeclareLaunchArgument(
                "nav_cancel_goal_service", default_value="/nav_command_server/cancel_goal"
            ),
            DeclareLaunchArgument(
                "nav_brake_service", default_value="/nav_command_server/brake"
            ),
            DeclareLaunchArgument(
                "nav_set_manual_mode_service",
                default_value="/nav_command_server/set_manual_mode",
            ),
            DeclareLaunchArgument(
                "teleop_cmd_topic",
                default_value="/cmd_vel_teleop",
            ),
            DeclareLaunchArgument(
                "nav_get_state_service", default_value="/nav_command_server/get_state"
            ),
            DeclareLaunchArgument(
                "nav_snapshot_service",
                default_value="/nav_snapshot_server/get_nav_snapshot",
            ),
            DeclareLaunchArgument(
                "nav_telemetry_topic",
                default_value="/nav_command_server/telemetry",
            ),
            DeclareLaunchArgument(
                "camera_pan_service",
                default_value="/camara/camera_pan",
            ),
            DeclareLaunchArgument(
                "camera_status_service",
                default_value="/camara/camera_status",
            ),
            DeclareLaunchArgument("request_timeout_s", default_value="5.0"),
            DeclareLaunchArgument("snapshot_request_timeout_s", default_value="2.0"),
            DeclareLaunchArgument("set_zones_timeout_s", default_value="12.0"),
            DeclareLaunchArgument("set_goal_timeout_s", default_value="12.0"),
            Node(
                package="map_tools",
                executable="web_zone_server",
                name="web_zone_server",
                output="screen",
                parameters=[
                    {
                        "ws_host": ws_host,
                        "ws_port": ws_port,
                        "gps_topic": gps_topic,
                        "map_frame": map_frame,
                        "zones_set_geojson_service": zones_set_geojson_service,
                        "zones_get_state_service": zones_get_state_service,
                        "zones_reload_service": zones_reload_service,
                        "nav_set_goal_service": nav_set_goal_service,
                        "nav_cancel_goal_service": nav_cancel_goal_service,
                        "nav_brake_service": nav_brake_service,
                        "nav_set_manual_mode_service": nav_set_manual_mode_service,
                        "nav_get_state_service": nav_get_state_service,
                        "teleop_cmd_topic": teleop_cmd_topic,
                        "nav_snapshot_service": nav_snapshot_service,
                        "nav_telemetry_topic": nav_telemetry_topic,
                        "camera_pan_service": camera_pan_service,
                        "camera_status_service": camera_status_service,
                        "request_timeout_s": request_timeout_s,
                        "snapshot_request_timeout_s": snapshot_request_timeout_s,
                        "set_zones_timeout_s": set_zones_timeout_s,
                        "set_goal_timeout_s": set_goal_timeout_s,
                    }
                ],
            ),
        ]
    )
