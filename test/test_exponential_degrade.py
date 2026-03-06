import numpy as np

from map_tools.mask_utils import exponential_gradient_from_core


def test_core_is_preserved_as_lethal_after_composition():
    core = np.zeros((41, 41), dtype=np.uint8)
    core[18:23, 18:23] = 100

    gradient = exponential_gradient_from_core(
        core_mask=core,
        resolution=0.1,
        radius_m=1.0,
        edge_cost=12,
        min_cost=1,
        use_l2=True,
    )
    composed = np.maximum(gradient, core)
    composed[core > 0] = 100

    assert np.all(composed[core > 0] == 100)


def test_gradient_monotonicity_along_line():
    core = np.zeros((81, 81), dtype=np.uint8)
    cy, cx = 40, 40
    core[cy, cx] = 100

    gradient = exponential_gradient_from_core(
        core_mask=core,
        resolution=0.1,
        radius_m=2.0,
        edge_cost=12,
        min_cost=1,
        use_l2=True,
    )

    # Distances from 0.1 m to 0.5 m at 0.1 m/cell.
    samples = [int(gradient[cy, cx + i]) for i in range(1, 6)]
    assert all(samples[i] <= samples[i - 1] for i in range(1, len(samples)))


def test_cost_near_radius_matches_edge_cost_tolerance():
    core = np.zeros((121, 121), dtype=np.uint8)
    cy, cx = 60, 60
    core[cy, cx] = 100

    radius_m = 1.0
    edge_cost = 20
    gradient = exponential_gradient_from_core(
        core_mask=core,
        resolution=0.1,
        radius_m=radius_m,
        edge_cost=edge_cost,
        min_cost=1,
        use_l2=True,
    )

    # Point roughly at 1.0 m from the core center.
    edge_sample = int(gradient[cy, cx + 10])
    assert 10 <= edge_sample <= 30


def test_outside_radius_is_zero():
    core = np.zeros((121, 121), dtype=np.uint8)
    cy, cx = 60, 60
    core[cy, cx] = 100

    gradient = exponential_gradient_from_core(
        core_mask=core,
        resolution=0.1,
        radius_m=1.0,
        edge_cost=12,
        min_cost=1,
        use_l2=True,
    )

    assert int(gradient[cy, cx + 15]) == 0


def test_union_behaves_like_max_of_individual_fields():
    core_a = np.zeros((81, 81), dtype=np.uint8)
    core_b = np.zeros((81, 81), dtype=np.uint8)
    core_a[40, 30] = 100
    core_b[40, 50] = 100
    core_union = np.maximum(core_a, core_b)

    params = {
        "resolution": 0.1,
        "radius_m": 2.0,
        "edge_cost": 12,
        "min_cost": 1,
        "use_l2": True,
    }
    grad_a = exponential_gradient_from_core(core_mask=core_a, **params)
    grad_b = exponential_gradient_from_core(core_mask=core_b, **params)
    grad_union = exponential_gradient_from_core(core_mask=core_union, **params)

    combined = np.maximum(grad_a, grad_b)
    assert np.array_equal(grad_union, combined)
