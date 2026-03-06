import numpy as np

from map_tools.mask_utils import rasterize_polygons_core


def test_rasterize_core_fills_polygon():
    zone = {
        "id": "zone_a",
        "enabled": True,
        "polygon_xy": [
            {"x": 1.0, "y": 1.0},
            {"x": 2.0, "y": 1.0},
            {"x": 2.0, "y": 2.0},
            {"x": 1.0, "y": 2.0},
        ],
    }
    core_mask, clipped, outside = rasterize_polygons_core(
        zones_xy=[zone],
        width=50,
        height=50,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
    )

    assert int(np.max(core_mask)) == 100
    assert clipped == {}
    assert outside == []


def test_rasterize_reports_outside_zone():
    outside_zone = {
        "id": "zone_out",
        "enabled": True,
        "polygon_xy": [
            {"x": 20.0, "y": 20.0},
            {"x": 21.0, "y": 20.0},
            {"x": 21.0, "y": 21.0},
        ],
    }
    core_mask, clipped, outside = rasterize_polygons_core(
        zones_xy=[outside_zone],
        width=100,
        height=100,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
    )

    assert int(np.max(core_mask)) == 0
    assert clipped == {}
    assert outside == ["zone_out"]


def test_rasterize_reports_clipped_vertices():
    partially_outside = {
        "id": "zone_clip",
        "enabled": True,
        "polygon_xy": [
            {"x": -1.0, "y": 1.0},
            {"x": 1.0, "y": 1.0},
            {"x": 1.0, "y": 3.0},
            {"x": -1.0, "y": 3.0},
        ],
    }
    core_mask, clipped, outside = rasterize_polygons_core(
        zones_xy=[partially_outside],
        width=100,
        height=100,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
    )

    assert int(np.max(core_mask)) == 100
    assert outside == []
    assert "zone_clip" in clipped
    assert clipped["zone_clip"] > 0
