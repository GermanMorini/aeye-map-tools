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
