import math
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np


def exponential_gradient_from_core(
    core_mask: np.ndarray,
    resolution: float,
    radius_m: float,
    edge_cost: int,
    min_cost: int,
    use_l2: bool,
) -> np.ndarray:
    """
    Build an exponential cost gradient around the lethal core mask.

    Parameters assume:
    - core_mask: uint8 matrix with lethal core marked as 100 and outside as 0.
    - resolution: meters/cell.
    - radius_m: gradient radius in meters.
    - edge_cost: target cost near radius edge (1..99).
    - min_cost: minimum non-zero gradient cost (1..99).
    """
    if (
        core_mask.size == 0
        or radius_m <= 0.0
        or resolution <= 0.0
        or edge_cost <= 0
        or edge_cost > 99
    ):
        return np.zeros_like(core_mask, dtype=np.uint8)

    outside = np.where(core_mask == 0, 255, 0).astype(np.uint8)
    distance_type = cv2.DIST_L2 if use_l2 else cv2.DIST_L1
    dist_px = cv2.distanceTransform(outside, distance_type, 3)
    dist_m = dist_px * float(resolution)

    in_band = (dist_m > 0.0) & (dist_m <= radius_m)
    gradient_mask = np.zeros_like(core_mask, dtype=np.uint8)
    if not np.any(in_band):
        return gradient_mask

    k = math.log(99.0 / float(edge_cost)) / float(radius_m)
    costs = np.rint(99.0 * np.exp(-k * dist_m[in_band])).astype(np.int16)
    costs = np.clip(costs, int(min_cost), 99)
    gradient_mask[in_band] = costs.astype(np.uint8)
    return gradient_mask


def rasterize_polygons_core(
    zones_xy: List[Dict[str, Any]],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> Tuple[np.ndarray, Dict[str, int], List[str]]:
    """
    Rasterize enabled polygons as lethal core (100) in a fixed grid.

    Returns:
    - core_mask: uint8 [height, width]
    - clipped_vertices: zone_id -> clipped vertex count
    - outside_zone_ids: zones whose bbox does not intersect the grid
    """
    core_mask = np.zeros((height, width), dtype=np.uint8)
    clipped_vertices: Dict[str, int] = {}
    outside_zone_ids: List[str] = []

    if width <= 0 or height <= 0 or resolution <= 0.0:
        return core_mask, clipped_vertices, outside_zone_ids

    min_x = float(origin_x)
    min_y = float(origin_y)
    max_x = min_x + (float(width) * float(resolution))
    max_y = min_y + (float(height) * float(resolution))

    for zone in zones_xy:
        if zone.get("enabled", True) is False:
            continue
        polygon_xy = zone.get("polygon_xy", [])
        if len(polygon_xy) < 3:
            continue
        zone_id = str(zone.get("id", "")) or "unnamed_zone"

        xs: List[float] = []
        ys: List[float] = []
        for p in polygon_xy:
            xs.append(float(p["x"]))
            ys.append(float(p["y"]))

        if max(xs) < min_x or min(xs) > max_x or max(ys) < min_y or min(ys) > max_y:
            outside_zone_ids.append(zone_id)
            continue

        pts: List[List[int]] = []
        clipped = 0
        for p in polygon_xy:
            x = float(p["x"])
            y = float(p["y"])
            col = int(np.floor((x - origin_x) / resolution))
            row = int(np.floor((y - origin_y) / resolution))
            original_col = col
            original_row = row

            if col < 0:
                col = 0
            elif col >= width:
                col = width - 1
            if row < 0:
                row = 0
            elif row >= height:
                row = height - 1

            if col != original_col or row != original_row:
                clipped += 1

            img_row = height - 1 - row
            pts.append([col, img_row])

        if len(pts) >= 3:
            arr = np.array([pts], dtype=np.int32)
            cv2.fillPoly(core_mask, arr, 100)

        if clipped > 0:
            clipped_vertices[zone_id] = clipped

    return core_mask, clipped_vertices, outside_zone_ids
