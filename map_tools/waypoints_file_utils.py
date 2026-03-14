from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import math
import yaml


def _to_finite_float(value: Any) -> Optional[float]:
    try:
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out):
        return None
    return out


def normalize_waypoint(item: Any, index: int) -> Tuple[Optional[Dict[str, float]], str]:
    if not isinstance(item, dict):
        return None, f"waypoint[{index}] must be an object"

    lat = _to_finite_float(item.get("lat", item.get("latitude")))
    lon = _to_finite_float(item.get("lon", item.get("longitude")))
    yaw = _to_finite_float(item.get("yaw_deg", item.get("yaw", 0.0)))

    if lat is None or lon is None or yaw is None:
        return None, f"invalid waypoint[{index}] values"
    return {"lat": lat, "lon": lon, "yaw_deg": yaw}, ""


def normalize_waypoints(waypoints_raw: Any) -> Tuple[Optional[List[Dict[str, float]]], str]:
    if not isinstance(waypoints_raw, list) or len(waypoints_raw) == 0:
        return None, "waypoints must be a non-empty list"

    out: List[Dict[str, float]] = []
    for idx, item in enumerate(waypoints_raw):
        normalized, err = normalize_waypoint(item, idx)
        if normalized is None:
            return None, err
        out.append(normalized)
    return out, ""


def build_waypoints_yaml_doc(waypoints: List[Dict[str, float]]) -> Dict[str, Any]:
    return {
        "waypoints": [
            {
                "latitude": float(wp["lat"]),
                "longitude": float(wp["lon"]),
                "yaw": float(wp.get("yaw_deg", 0.0)),
            }
            for wp in waypoints
        ]
    }


def parse_waypoints_yaml_text(yaml_text: str) -> Tuple[Optional[List[Dict[str, float]]], str]:
    try:
        raw = yaml.safe_load(yaml_text)
    except Exception as exc:
        return None, f"invalid yaml: {exc}"
    if not isinstance(raw, dict):
        return None, "yaml root must be a map/object"
    return normalize_waypoints(raw.get("waypoints"))


def save_waypoints_yaml_file(
    file_path: Path, waypoints: List[Dict[str, float]]
) -> Tuple[bool, str, int]:
    normalized, err = normalize_waypoints(waypoints)
    if normalized is None:
        return False, err, 0

    try:
        file_path.parent.mkdir(parents=True, exist_ok=True)
    except Exception as exc:
        return False, f"failed creating output dir: {exc}", 0

    data = build_waypoints_yaml_doc(normalized)
    try:
        with file_path.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(data, handle, sort_keys=False)
    except Exception as exc:
        return False, f"failed writing waypoints file: {exc}", 0

    return True, "", len(normalized)


def load_waypoints_yaml_file(
    file_path: Path,
) -> Tuple[bool, str, List[Dict[str, float]]]:
    if not file_path.exists():
        return False, f"waypoints file not found: {file_path}", []

    try:
        with file_path.open("r", encoding="utf-8") as handle:
            raw_text = handle.read()
    except Exception as exc:
        return False, f"failed reading waypoints file: {exc}", []

    waypoints, err = parse_waypoints_yaml_text(raw_text)
    if waypoints is None:
        return False, err, []
    return True, "", waypoints
