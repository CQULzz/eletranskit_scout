import numpy as np

from ht_traversability_ros2.gridmap_codec import decode_layer, encode_layer


def test_gridmap_layer_round_trip():
    source = np.arange(12, dtype=np.float32).reshape(3, 4)
    message = encode_layer(source)
    result = decode_layer("test", message)
    np.testing.assert_array_equal(result, source)


def test_gridmap_layer_round_trip_preserves_nan():
    source = np.array([[1.0, np.nan], [2.0, 3.0]], dtype=np.float32)
    result = decode_layer("test", encode_layer(source))
    np.testing.assert_array_equal(np.isnan(result), np.isnan(source))
    np.testing.assert_allclose(np.nan_to_num(result), np.nan_to_num(source))
