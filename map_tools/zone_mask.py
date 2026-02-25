#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class MaskSpec:
    frame_id: str
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    default_value: int = 0
    no_go_value: int = 100


def world_to_cell(x: float, y: float, spec: MaskSpec) -> tuple[int, int]:
    col = int((x - spec.origin_x) / spec.resolution)
    row = int((y - spec.origin_y) / spec.resolution)
    return (col, row)


def cell_center(col: int, row: int, spec: MaskSpec) -> tuple[float, float]:
    x = spec.origin_x + (col + 0.5) * spec.resolution
    y = spec.origin_y + (row + 0.5) * spec.resolution
    return (x, y)


def _point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        intersects = ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
        )
        if intersects:
            inside = not inside
        j = i
    return inside


def _polygon_bounds_cells(polygon: list[tuple[float, float]], spec: MaskSpec) -> tuple[int, int, int, int]:
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)

    min_col, min_row = world_to_cell(min_x, min_y, spec)
    max_col, max_row = world_to_cell(max_x, max_y, spec)

    min_col = max(0, min(spec.width - 1, min_col))
    max_col = max(0, min(spec.width - 1, max_col))
    min_row = max(0, min(spec.height - 1, min_row))
    max_row = max(0, min(spec.height - 1, max_row))
    return (min_col, max_col, min_row, max_row)


def rasterize_no_go_zones(
    polygons: Iterable[list[tuple[float, float]]],
    spec: MaskSpec,
) -> list[int]:
    values = [int(spec.default_value)] * (spec.width * spec.height)
    no_go_value = int(spec.no_go_value)

    for polygon in polygons:
        if len(polygon) < 3:
            continue

        min_col, max_col, min_row, max_row = _polygon_bounds_cells(polygon, spec)
        for row in range(min_row, max_row + 1):
            for col in range(min_col, max_col + 1):
                x, y = cell_center(col, row, spec)
                if _point_in_polygon(x, y, polygon):
                    index = row * spec.width + col
                    values[index] = no_go_value

    return values
