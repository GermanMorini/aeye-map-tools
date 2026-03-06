from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws_host = LaunchConfiguration("ws_host")
    ws_port = LaunchConfiguration("ws_port")
    fromll_service = LaunchConfiguration("fromll_service")
    global_costmap_service = LaunchConfiguration("global_costmap_service")
    gps_topic = LaunchConfiguration("gps_topic")
    ptz_enabled = LaunchConfiguration("ptz_enabled")
    ptz_camera_ip = LaunchConfiguration("ptz_camera_ip")
    ptz_camera_user = LaunchConfiguration("ptz_camera_user")
    ptz_camera_pass = LaunchConfiguration("ptz_camera_pass")
    ptz_camera_channel = LaunchConfiguration("ptz_camera_channel")
    ptz_timeout_s = LaunchConfiguration("ptz_timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("ws_port", default_value="8765"),
            DeclareLaunchArgument("fromll_service", default_value="/fromLL"),
            DeclareLaunchArgument(
                "global_costmap_service", default_value="/global_costmap/get_costmap"
            ),
            DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("ptz_enabled", default_value="true"),
            DeclareLaunchArgument("ptz_camera_ip", default_value="192.168.1.64"),
            DeclareLaunchArgument("ptz_camera_user", default_value="admin"),
            DeclareLaunchArgument("ptz_camera_pass", default_value="teamcit2024"),
            DeclareLaunchArgument("ptz_camera_channel", default_value="1"),
            DeclareLaunchArgument("ptz_timeout_s", default_value="1.5"),
            Node(
                package="map_tools",
                executable="web_zone_server",
                name="web_zone_server",
                output="screen",
                parameters=[
                    {
                        "ws_host": ws_host,
                        "ws_port": ws_port,
                        "fromll_service": fromll_service,
                        "global_costmap_service": global_costmap_service,
                        "mask_topic": "/keepout_filter_mask",
                        "filter_info_topic": "/costmap_filter_info",
                        "gps_topic": gps_topic,
                        "ptz_enabled": ptz_enabled,
                        "ptz_camera_ip": ptz_camera_ip,
                        "ptz_camera_user": ptz_camera_user,
                        "ptz_camera_pass": ptz_camera_pass,
                        "ptz_camera_channel": ptz_camera_channel,
                        "ptz_timeout_s": ptz_timeout_s,
                    }
                ],
            ),
        ]
    )
