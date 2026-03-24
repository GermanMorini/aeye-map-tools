from pathlib import Path


def test_no_go_editor_launch_exposes_fromll_output_frame_for_zones_manager() -> None:
    launch_path = Path(__file__).resolve().parents[1] / "launch" / "no_go_editor.launch.py"
    launch_contents = launch_path.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument(\n                "zones_fromll_output_frame", default_value="map"' in launch_contents
    assert '"fromll_target_frame": map_frame' in launch_contents
    assert '"fromll_output_frame": zones_fromll_output_frame' in launch_contents
