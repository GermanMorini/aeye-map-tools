from pathlib import Path


def test_no_go_editor_launch_exposes_fromll_output_frame_for_zones_manager() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "no_go_editor.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n                "zones_fromll_output_frame", default_value="map"' in launch_contents
    assert '"fromll_target_frame": map_frame' in launch_contents
    assert '"fromll_output_frame": zones_fromll_output_frame' in launch_contents


def test_no_go_editor_launch_exposes_nav_set_datum_service_for_web_gateway() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "no_go_editor.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert (
        'DeclareLaunchArgument(\n'
        '                "nav_set_datum_service",\n'
        '                default_value="/datum_setter/set_datum",\n'
        "            )"
    ) in launch_contents
    assert '"nav_set_datum_service": nav_set_datum_service' in launch_contents


def test_no_go_editor_launch_exposes_sensor_info_topics_for_web_gateway() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "no_go_editor.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered")' in launch_contents
    assert (
        'DeclareLaunchArgument(\n'
        '                "robot_heading_topic", default_value="/odometry/global"\n'
        "            )"
    ) in launch_contents
    assert 'DeclareLaunchArgument("imu_topic", default_value="/imu/data")' in launch_contents
    assert 'DeclareLaunchArgument("velocity_topic", default_value="/velocity")' in launch_contents
    assert 'DeclareLaunchArgument("fix_type_topic", default_value="/gps/fix_type")' in launch_contents
    assert 'DeclareLaunchArgument("rtk_status_topic", default_value="/gps/rtk_status")' in launch_contents
    assert 'DeclareLaunchArgument("rtcm_age_topic", default_value="/gps/rtcm_age_s")' in launch_contents
    assert 'DeclareLaunchArgument("rtcm_count_topic", default_value="/gps/rtcm_received_count")' in launch_contents
    assert 'DeclareLaunchArgument("gps_raw_topic", default_value="/mavros_node/gps1/raw")' in launch_contents
    assert '"imu_topic": imu_topic' in launch_contents
    assert '"velocity_topic": velocity_topic' in launch_contents
    assert '"odom_topic": odom_topic' in launch_contents
    assert '"robot_heading_topic": robot_heading_topic' in launch_contents
    assert '"fix_type_topic": fix_type_topic' in launch_contents
    assert '"rtk_status_topic": rtk_status_topic' in launch_contents
    assert '"rtcm_age_topic": rtcm_age_topic' in launch_contents
    assert '"rtcm_count_topic": rtcm_count_topic' in launch_contents
    assert '"gps_raw_topic": gps_raw_topic' in launch_contents
    assert '"rtk_source_status_topic": rtk_source_status_topic' in launch_contents
    assert '"nav_get_datum_service": nav_get_datum_service' in launch_contents


def test_no_go_editor_launch_exposes_control_lock_services() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "no_go_editor.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert (
        'DeclareLaunchArgument(\n'
        '                "nav_set_control_lock_service",\n'
        '                default_value="/nav_command_server/set_control_lock",\n'
        "            )"
    ) in launch_contents
    assert (
        'DeclareLaunchArgument(\n'
        '                "nav_touch_control_heartbeat_service",\n'
        '                default_value="/nav_command_server/touch_control_heartbeat",\n'
        "            )"
    ) in launch_contents
    assert '"set_control_lock_service": nav_set_control_lock_service' in launch_contents
    assert '"touch_control_heartbeat_service": nav_touch_control_heartbeat_service' in launch_contents
    assert '"nav_set_control_lock_service": nav_set_control_lock_service' in launch_contents
    assert '"nav_touch_control_heartbeat_service": nav_touch_control_heartbeat_service' in launch_contents


def test_no_go_editor_launch_uses_relaxed_snapshot_timeout_default() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "no_go_editor.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("snapshot_request_timeout_s", default_value="5.0")' in launch_contents
