from pathlib import Path

from map_tools.waypoints_file_utils import (
    load_waypoints_yaml_file,
    parse_waypoints_yaml_text,
    save_waypoints_yaml_file,
)


def test_parse_waypoints_yaml_text_canonical():
    text = """
waypoints:
  - latitude: -31.0
    longitude: -64.0
    yaw: 10.0
  - latitude: -31.1
    longitude: -64.1
    yaw: 20.0
"""
    waypoints, err = parse_waypoints_yaml_text(text)
    assert err == ""
    assert waypoints == [
        {"lat": -31.0, "lon": -64.0, "yaw_deg": 10.0},
        {"lat": -31.1, "lon": -64.1, "yaw_deg": 20.0},
    ]


def test_parse_waypoints_yaml_text_variant_keys():
    text = """
waypoints:
  - lat: -31.2
    lon: -64.2
    yaw_deg: 30.0
"""
    waypoints, err = parse_waypoints_yaml_text(text)
    assert err == ""
    assert waypoints == [{"lat": -31.2, "lon": -64.2, "yaw_deg": 30.0}]


def test_parse_waypoints_yaml_text_invalid():
    waypoints, err = parse_waypoints_yaml_text("waypoints: [")
    assert waypoints is None
    assert "invalid yaml" in err


def test_save_and_load_waypoints_yaml_file(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    src = [
        {"lat": -31.4, "lon": -64.4, "yaw_deg": 5.0},
        {"lat": -31.5, "lon": -64.5, "yaw_deg": -15.0},
    ]
    ok_save, err_save, count = save_waypoints_yaml_file(file_path, src)
    assert ok_save
    assert err_save == ""
    assert count == 2

    ok_load, err_load, loaded = load_waypoints_yaml_file(file_path)
    assert ok_load
    assert err_load == ""
    assert loaded == src

    raw_text = file_path.read_text(encoding="utf-8")
    assert "latitude" in raw_text
    assert "longitude" in raw_text
    assert "yaw" in raw_text


def test_load_waypoints_yaml_file_missing(tmp_path: Path):
    missing = tmp_path / "missing.yaml"
    ok, err, waypoints = load_waypoints_yaml_file(missing)
    assert not ok
    assert "not found" in err
    assert waypoints == []


def test_save_waypoints_yaml_file_rejects_empty(tmp_path: Path):
    file_path = tmp_path / "saved_waypoints.yaml"
    ok, err, count = save_waypoints_yaml_file(file_path, [])
    assert not ok
    assert "non-empty" in err
    assert count == 0
