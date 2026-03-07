from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    gps_topic = LaunchConfiguration("gps_topic")
    map_frame = LaunchConfiguration("map_frame")

    keepout_set_zones_service = LaunchConfiguration("keepout_set_zones_service")
    keepout_get_state_service = LaunchConfiguration("keepout_get_state_service")

    nav_set_goal_service = LaunchConfiguration("nav_set_goal_service")
    nav_cancel_goal_service = LaunchConfiguration("nav_cancel_goal_service")
    nav_brake_service = LaunchConfiguration("nav_brake_service")
    nav_set_manual_mode_service = LaunchConfiguration("nav_set_manual_mode_service")
    nav_set_manual_cmd_service = LaunchConfiguration("nav_set_manual_cmd_service")
    nav_get_state_service = LaunchConfiguration("nav_get_state_service")

    nav_snapshot_service = LaunchConfiguration("nav_snapshot_service")
    nav_telemetry_topic = LaunchConfiguration("nav_telemetry_topic")

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
                "keepout_set_zones_service", default_value="/keepout_manager/set_zones"
            ),
            DeclareLaunchArgument(
                "keepout_get_state_service", default_value="/keepout_manager/get_state"
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
                "nav_set_manual_cmd_service",
                default_value="/nav_command_server/set_manual_cmd",
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
                        "keepout_set_zones_service": keepout_set_zones_service,
                        "keepout_get_state_service": keepout_get_state_service,
                        "nav_set_goal_service": nav_set_goal_service,
                        "nav_cancel_goal_service": nav_cancel_goal_service,
                        "nav_brake_service": nav_brake_service,
                        "nav_set_manual_mode_service": nav_set_manual_mode_service,
                        "nav_set_manual_cmd_service": nav_set_manual_cmd_service,
                        "nav_get_state_service": nav_get_state_service,
                        "nav_snapshot_service": nav_snapshot_service,
                        "nav_telemetry_topic": nav_telemetry_topic,
                        "request_timeout_s": request_timeout_s,
                        "snapshot_request_timeout_s": snapshot_request_timeout_s,
                        "set_zones_timeout_s": set_zones_timeout_s,
                        "set_goal_timeout_s": set_goal_timeout_s,
                    }
                ],
            ),
        ]
    )
