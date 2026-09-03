"""ROS-independent preprocessing and output placement utilities."""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


RECEPTIVE_FIELD = 45
DIRECTION_COUNT = 8


def validate_direction_order(order: Sequence[int]) -> Tuple[int, ...]:
    result = tuple(int(value) for value in order)
    if len(result) != DIRECTION_COUNT or sorted(result) != list(range(DIRECTION_COUNT)):
        raise ValueError("direction_channel_order must be a permutation of 0..7")
    return result


def nearest_fill_nan(height: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Fill NaNs with the nearest observed height and return the original mask.

    Filling is only for numerical inference.  The original validity mask must
    still be used to invalidate poorly observed output windows.
    """

    array = np.asarray(height, dtype=np.float32)
    if array.ndim != 2:
        raise ValueError(f"Expected a 2-D elevation grid, got shape {array.shape}")
    finite = np.isfinite(array)
    if not finite.any():
        raise ValueError("Elevation grid contains no finite cells")
    if finite.all():
        return np.ascontiguousarray(array), finite

    try:
        from scipy.ndimage import distance_transform_edt
    except ImportError as exc:
        raise RuntimeError(
            "python3-scipy is required to fill unknown elevation cells"
        ) from exc

    nearest = distance_transform_edt(~finite, return_distances=False, return_indices=True)
    filled = array.copy()
    filled[~finite] = array[tuple(axis[~finite] for axis in nearest)]
    return np.ascontiguousarray(filled, dtype=np.float32), finite


def valid_window_ratio(valid_mask: np.ndarray, window: int = RECEPTIVE_FIELD) -> np.ndarray:
    """Fraction of observed input cells in every valid convolution window."""

    mask = np.asarray(valid_mask, dtype=np.float32)
    if mask.ndim != 2:
        raise ValueError("valid_mask must be 2-D")
    rows, cols = mask.shape
    if rows < window or cols < window:
        raise ValueError(f"Input {rows}x{cols} is smaller than {window}x{window}")

    integral = np.pad(mask, ((1, 0), (1, 0)), mode="constant").cumsum(0).cumsum(1)
    counts = (
        integral[window:, window:]
        - integral[:-window, window:]
        - integral[window:, :-window]
        + integral[:-window, :-window]
    )
    return counts / float(window * window)


def embed_center(values: np.ndarray, output_shape: tuple[int, int], fill: float = np.nan) -> np.ndarray:
    """Place a smaller 2-D array at the exact center of the output grid."""

    source = np.asarray(values, dtype=np.float32)
    if source.ndim != 2:
        raise ValueError("Only 2-D layers can be embedded")
    rows, cols = output_shape
    if source.shape[0] > rows or source.shape[1] > cols:
        raise ValueError("Source layer is larger than output grid")
    top = (rows - source.shape[0]) // 2
    left = (cols - source.shape[1]) // 2
    bottom = top + source.shape[0]
    right = left + source.shape[1]
    result = np.full((rows, cols), fill, dtype=np.float32)
    result[top:bottom, left:right] = source
    return result


def build_output_layers(
    probabilities: np.ndarray,
    valid_output: np.ndarray,
    full_shape: tuple[int, int],
) -> dict[str, np.ndarray]:
    """Create eight directional layers plus best-direction diagnostics."""

    probs = np.asarray(probabilities, dtype=np.float32)
    valid = np.asarray(valid_output, dtype=bool)
    if probs.ndim != 3 or probs.shape[0] != DIRECTION_COUNT:
        raise ValueError(f"Expected probabilities [8,H,W], got {probs.shape}")
    if probs.shape[1:] != valid.shape:
        raise ValueError("Probability and validity shapes do not match")

    masked = np.where(valid[None, :, :], probs, np.nan).astype(np.float32)
    best_dir = np.argmax(probs, axis=0).astype(np.float32)
    best_prob = np.max(probs, axis=0).astype(np.float32)
    best_dir[~valid] = np.nan
    best_prob[~valid] = np.nan

    layers = {
        f"zt_dir_{index}": embed_center(masked[index], full_shape)
        for index in range(DIRECTION_COUNT)
    }
    layers["zt_best_dir"] = embed_center(best_dir, full_shape)
    layers["zt_best_prob"] = embed_center(best_prob, full_shape)
    layers["zt_valid"] = embed_center(valid.astype(np.float32), full_shape, fill=0.0)
    return layers
