from map_tools.zone_mask import MaskSpec
from map_tools.zone_mask import rasterize_no_go_zones


def test_rasterize_single_square_marks_cells() -> None:
    spec = MaskSpec(
        frame_id="map",
        resolution=1.0,
        width=10,
        height=10,
        origin_x=0.0,
        origin_y=0.0,
        default_value=0,
        no_go_value=100,
    )
    square = [(2.0, 2.0), (5.0, 2.0), (5.0, 5.0), (2.0, 5.0)]

    data = rasterize_no_go_zones([square], spec)

    assert len(data) == 100
    inside_idx = 3 * spec.width + 3
    outside_idx = 0
    assert data[inside_idx] == 100
    assert data[outside_idx] == 0


def test_rasterize_respects_enabled_pre_filtering_contract() -> None:
    spec = MaskSpec(
        frame_id="map",
        resolution=0.5,
        width=6,
        height=6,
        origin_x=0.0,
        origin_y=0.0,
        default_value=0,
        no_go_value=100,
    )

    triangle = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    data = rasterize_no_go_zones([triangle], spec)

    assert any(value == 100 for value in data)
