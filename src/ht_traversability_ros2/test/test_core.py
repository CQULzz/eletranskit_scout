import numpy as np
import pytest

from ht_traversability_ros2.core import (
    build_conservative_traversability_layer,
    build_output_layers,
    embed_center,
    nearest_fill_nan,
    valid_window_ratio,
    validate_direction_order,
)


def test_valid_window_ratio_shape_and_value():
    mask = np.ones((250, 250), dtype=bool)
    result = valid_window_ratio(mask, 51)
    assert result.shape == (200, 200)
    np.testing.assert_allclose(result, 1.0)


def test_valid_window_ratio_detects_one_missing_cell():
    mask = np.ones((51, 51), dtype=bool)
    mask[0, 0] = False
    result = valid_window_ratio(mask, 51)
    np.testing.assert_allclose(result, (51 * 51 - 1) / (51 * 51))


def test_nearest_fill_preserves_mask_and_fills_nan():
    height = np.array([[1.0, np.nan], [2.0, 3.0]], dtype=np.float32)
    filled, finite = nearest_fill_nan(height)
    assert finite.tolist() == [[True, False], [True, True]]
    assert np.isfinite(filled).all()
    assert filled[0, 0] == 1.0


def test_nearest_fill_rejects_all_nan():
    with pytest.raises(ValueError, match="no finite"):
        nearest_fill_nan(np.full((51, 51), np.nan, dtype=np.float32))


def test_embed_center_250_from_200():
    source = np.ones((200, 200), dtype=np.float32)
    result = embed_center(source, (250, 250))
    assert result.shape == (250, 250)
    assert np.isnan(result[:25]).all()
    assert np.isnan(result[-25:]).all()
    assert np.isfinite(result[25:-25, 25:-25]).all()


def test_output_layers_and_invalid_mask():
    probs = np.stack(
        [np.full((2, 3), index / 10.0, dtype=np.float32) for index in range(8)]
    )
    valid = np.ones((2, 3), dtype=bool)
    valid[0, 0] = False
    layers = build_output_layers(probs, valid, (4, 5))
    assert list(layers) == [
        "ht_dir_0",
        "ht_dir_1",
        "ht_dir_2",
        "ht_dir_3",
        "ht_dir_4",
        "ht_dir_5",
        "ht_dir_6",
        "ht_dir_7",
        "ht_best_dir",
        "ht_best_prob",
        "ht_valid",
    ]
    assert np.nanmax(layers["ht_best_dir"]) == 7.0
    assert np.nanmax(layers["ht_best_prob"]) == np.float32(0.7)
    assert np.count_nonzero(layers["ht_valid"]) == 5


def test_direction_order_validation():
    assert validate_direction_order(range(8)) == tuple(range(8))
    with pytest.raises(ValueError, match="permutation"):
        validate_direction_order([0] * 8)


def test_conservative_layer_requires_all_eight_directions_at_threshold():
    probabilities = np.full((8, 2, 3), 0.8, dtype=np.float32)
    probabilities[3, 0, 1] = 0.49
    probabilities[7, 1, 2] = 0.5  # Equal to the threshold remains traversable.
    valid = np.ones((2, 3), dtype=bool)
    valid[1, 0] = False

    result = build_conservative_traversability_layer(
        probabilities, valid, (4, 5), threshold=0.5
    )

    expected_inner = np.array(
        [[1.0, 0.0, 1.0], [np.nan, 1.0, 1.0]], dtype=np.float32
    )
    assert result.shape == (4, 5)
    np.testing.assert_equal(result[1:3, 1:4], expected_inner)
    assert np.isnan(result[0]).all()
    assert np.isnan(result[-1]).all()


@pytest.mark.parametrize("threshold", [-0.01, 1.01, np.nan])
def test_conservative_layer_rejects_invalid_threshold(threshold):
    probabilities = np.ones((8, 1, 1), dtype=np.float32)
    with pytest.raises(ValueError, match="between 0 and 1"):
        build_conservative_traversability_layer(
            probabilities, np.ones((1, 1), dtype=bool), (1, 1), threshold
        )
