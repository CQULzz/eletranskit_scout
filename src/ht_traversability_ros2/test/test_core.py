import numpy as np
import pytest

from ht_traversability_ros2.core import (
    build_conservative_traversability_layer,
    build_direction_cost_layers,
    build_output_layers,
    embed_center,
    nearest_fill_nan,
    prepare_observation_height,
    add_unobserved_prior_layers,
    valid_window_ratio,
    validate_direction_order,
)


def test_initial_prior_has_free_costs_without_fabricated_model_scores():
    height = np.zeros((55, 55), np.float32)
    filled, observed = prepare_observation_height(height, np.zeros_like(height))
    assert not observed.any()
    assert (filled == 0).all()
    layers = {"ht_dir_0": np.full_like(height, np.nan),
              "ht_cost_dir_0": np.full_like(height, np.nan)}
    add_unobserved_prior_layers(layers, observed)
    assert np.isnan(layers["ht_dir_0"]).all()
    assert (layers["ht_cost_dir_0"] == 0).all()
    assert (layers["ht_prior_free"] == 1).all()
    assert (layers["ht_observed"] == 0).all()


def test_unobserved_minus_one_height_is_preserved_for_empty_model_context():
    height = np.full((55, 55), -1., np.float32)
    filled, observed = prepare_observation_height(height, np.zeros_like(height))
    assert (filled == -1).all()
    assert not observed.any()


def test_prior_zero_is_not_used_as_a_real_height_in_model_context():
    height = np.zeros((55, 55), np.float32)
    count = np.zeros_like(height)
    height[20, 20], count[20, 20] = 2., 1.
    filled, observed = prepare_observation_height(height, count)
    assert (filled == 2).all()
    assert observed.sum() == 1
    layers = {"ht_cost_dir_0": np.ones_like(height)}
    add_unobserved_prior_layers(layers, observed)
    assert layers["ht_cost_dir_0"][20, 20] == 1
    assert (layers["ht_cost_dir_0"][~observed] == 0).all()
    count[0, 0] = np.nan
    with pytest.raises(ValueError, match="observation_count"):
        prepare_observation_height(height, count)


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


def test_direction_cost_layers_are_binary_and_preserve_unknown():
    probabilities = np.stack(
        [np.full((2, 3), index / 10.0, dtype=np.float32) for index in range(8)]
    )
    valid = np.ones((2, 3), dtype=bool)
    valid[0, 0] = False

    layers = build_direction_cost_layers(probabilities, valid, (4, 5), threshold=0.5)

    assert list(layers) == [f"ht_cost_dir_{index}" for index in range(8)]
    for index in range(8):
        inner = layers[f"ht_cost_dir_{index}"][1:3, 1:4]
        assert np.isnan(inner[0, 0])
        np.testing.assert_equal(
            inner[valid], (probabilities[index][valid] < 0.5).astype(np.float32)
        )
        finite = inner[np.isfinite(inner)]
        assert set(np.unique(finite)).issubset({0.0, 1.0})


def test_direction_cost_threshold_equality_is_free():
    probabilities = np.full((8, 1, 2), 0.5, dtype=np.float32)
    probabilities[:, 0, 0] = np.nextafter(np.float32(0.5), np.float32(0.0))
    layers = build_direction_cost_layers(
        probabilities, np.ones((1, 2), dtype=bool), (1, 2), threshold=0.5
    )
    np.testing.assert_equal(layers["ht_cost_dir_0"], [[1.0, 0.0]])


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
