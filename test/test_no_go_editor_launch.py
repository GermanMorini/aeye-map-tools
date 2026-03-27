from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_no_go_editor_launch_supports_odom_topic_override() -> None:
    launch_contents = (PACKAGE_ROOT / "launch" / "no_go_editor.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'LaunchConfiguration("odom_topic")' in launch_contents
    assert 'DeclareLaunchArgument("odom_topic", default_value="/odometry/local")' in launch_contents
    assert '"odom_topic": odom_topic' in launch_contents
