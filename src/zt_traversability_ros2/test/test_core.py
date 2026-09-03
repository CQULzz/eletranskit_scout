import numpy as np

from zt_traversability_ros2.core import (
    build_output_layers,
    embed_center,
    valid_window_ratio,
    validate_direction_order,
)


def test_valid_window_ratio_shape_and_value():
    mask = np.ones((250, 250), dtype=bool)
    result = valid_window_ratio(mask, 45)
    assert result.shape == (206, 206)
    np.testing.assert_allclose(result, 1.0)


def test_valid_window_ratio_detects_one_missing_cell():
    mask = np.ones((45, 45), dtype=bool)
    mask[0, 0] = False
    result = valid_window_ratio(mask, 45)
    np.testing.assert_allclose(result, (45 * 45 - 1) / (45 * 45))


def test_embed_center_250_from_206():
    source = np.ones((206, 206), dtype=np.float32)
    result = embed_center(source, (250, 250))
    assert result.shape == (250, 250)
    assert np.isnan(result[:22]).all()
    assert np.isnan(result[-22:]).all()
    assert np.isfinite(result[22:-22, 22:-22]).all()


def test_output_layers():
    probs = np.stack(
        [np.full((2, 3), index / 10.0, dtype=np.float32) for index in range(8)]
    )
    valid = np.ones((2, 3), dtype=bool)
    valid[0, 0] = False
    layers = build_output_layers(probs, valid, (4, 5))
    assert list(layers) == [
        "zt_dir_0",
        "zt_dir_1",
        "zt_dir_2",
        "zt_dir_3",
        "zt_dir_4",
        "zt_dir_5",
        "zt_dir_6",
        "zt_dir_7",
        "zt_best_dir",
        "zt_best_prob",
        "zt_valid",
    ]
    assert np.nanmax(layers["zt_best_dir"]) == 7.0
    assert np.nanmax(layers["zt_best_prob"]) == np.float32(0.7)


def test_direction_order_validation():
    assert validate_direction_order(range(8)) == tuple(range(8))
    try:
        validate_direction_order([0] * 8)
    except ValueError:
        pass
    else:
        raise AssertionError("Invalid direction order was accepted")
