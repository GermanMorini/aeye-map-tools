#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ZoneValidationError(ValueError):
    """Raised when zone data does not match the expected schema."""


@dataclass(frozen=True)
class Zone:
    zone_id: str
    zone_type: str
    enabled: bool
    polygon: list[tuple[float, float]]


@dataclass(frozen=True)
class ZoneDocument:
    frame_id: str
    zones: list[Zone]


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _point_from_raw(point: Any, zone_id: str) -> tuple[float, float]:
    if not isinstance(point, list) or len(point) != 2:
        raise ZoneValidationError(f"Zone '{zone_id}' has invalid point format: {point!r}")
    x, y = point
    if not _is_number(x) or not _is_number(y):
        raise ZoneValidationError(f"Zone '{zone_id}' has non-numeric point: {point!r}")
    return (float(x), float(y))


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> bool:
    return (
        min(a[0], b[0]) <= c[0] <= max(a[0], b[0])
        and min(a[1], b[1]) <= c[1] <= max(a[1], b[1])
    )


def _segments_intersect(
    p1: tuple[float, float],
    q1: tuple[float, float],
    p2: tuple[float, float],
    q2: tuple[float, float],
) -> bool:
    o1 = _orientation(p1, q1, p2)
    o2 = _orientation(p1, q1, q2)
    o3 = _orientation(p2, q2, p1)
    o4 = _orientation(p2, q2, q1)

    if o1 == 0.0 and _on_segment(p1, q1, p2):
        return True
    if o2 == 0.0 and _on_segment(p1, q1, q2):
        return True
    if o3 == 0.0 and _on_segment(p2, q2, p1):
        return True
    if o4 == 0.0 and _on_segment(p2, q2, q1):
        return True
    return (o1 > 0.0) != (o2 > 0.0) and (o3 > 0.0) != (o4 > 0.0)


def _polygon_edges(points: list[tuple[float, float]]) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    edges: list[tuple[tuple[float, float], tuple[float, float]]] = []
    for idx, start in enumerate(points):
        end = points[(idx + 1) % len(points)]
        edges.append((start, end))
    return edges


def _is_self_intersecting(points: list[tuple[float, float]]) -> bool:
    edges = _polygon_edges(points)
    edge_count = len(edges)
    for i in range(edge_count):
        a_start, a_end = edges[i]
        for j in range(i + 1, edge_count):
            if j == i:
                continue
            if (j == i + 1) or (i == 0 and j == edge_count - 1):
                continue
            b_start, b_end = edges[j]
            if _segments_intersect(a_start, a_end, b_start, b_end):
                return True
    return False


def _polygon_area(points: list[tuple[float, float]]) -> float:
    area = 0.0
    for idx, (x1, y1) in enumerate(points):
        x2, y2 = points[(idx + 1) % len(points)]
        area += (x1 * y2) - (x2 * y1)
    return abs(area) * 0.5


def validate_document(raw_data: dict[str, Any]) -> ZoneDocument:
    if not isinstance(raw_data, dict):
        raise ZoneValidationError("Zone document root must be a mapping")

    frame_id = raw_data.get("frame_id", "map")
    if not isinstance(frame_id, str) or not frame_id:
        raise ZoneValidationError("frame_id must be a non-empty string")

    zones_raw = raw_data.get("zones", [])
    if not isinstance(zones_raw, list):
        raise ZoneValidationError("zones must be a list")

    zones: list[Zone] = []
    seen_ids: set[str] = set()
    for idx, zone_raw in enumerate(zones_raw):
        if not isinstance(zone_raw, dict):
            raise ZoneValidationError(f"Zone at index {idx} must be a mapping")

        zone_id = zone_raw.get("id")
        if not isinstance(zone_id, str) or not zone_id:
            raise ZoneValidationError(f"Zone at index {idx} has invalid id")
        if zone_id in seen_ids:
            raise ZoneValidationError(f"Duplicate zone id: '{zone_id}'")
        seen_ids.add(zone_id)

        zone_type = zone_raw.get("type", "no_go")
        if not isinstance(zone_type, str) or not zone_type:
            raise ZoneValidationError(f"Zone '{zone_id}' has invalid type")

        enabled = zone_raw.get("enabled", True)
        if not isinstance(enabled, bool):
            raise ZoneValidationError(f"Zone '{zone_id}' has invalid enabled flag")

        polygon_raw = zone_raw.get("polygon")
        if not isinstance(polygon_raw, list) or len(polygon_raw) < 3:
            raise ZoneValidationError(f"Zone '{zone_id}' polygon must contain at least 3 points")
        polygon = [_point_from_raw(point, zone_id) for point in polygon_raw]
        if _is_self_intersecting(polygon):
            raise ZoneValidationError(f"Zone '{zone_id}' polygon is self-intersecting")
        if _polygon_area(polygon) <= 1e-8:
            raise ZoneValidationError(f"Zone '{zone_id}' polygon area is too small")

        zones.append(Zone(zone_id=zone_id, zone_type=zone_type, enabled=enabled, polygon=polygon))

    return ZoneDocument(frame_id=frame_id, zones=zones)


def to_raw_document(document: ZoneDocument) -> dict[str, Any]:
    return {
        "frame_id": document.frame_id,
        "zones": [
            {
                "id": zone.zone_id,
                "type": zone.zone_type,
                "enabled": zone.enabled,
                "polygon": [[point[0], point[1]] for point in zone.polygon],
            }
            for zone in document.zones
        ],
    }


def load_zone_document(path: str) -> ZoneDocument:
    file_path = Path(path)
    if not file_path.exists():
        return ZoneDocument(frame_id="map", zones=[])
    with file_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    return validate_document(raw)


def save_zone_document(path: str, document: ZoneDocument) -> None:
    file_path = Path(path)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    raw_data = to_raw_document(document)
    with file_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(raw_data, handle, sort_keys=False)
