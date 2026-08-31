"""Minimal ROS GridMap Float32MultiArray codec."""

from __future__ import annotations

from array import array as float_array
import math

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


def decode_layer(name: str, message: Float32MultiArray) -> np.ndarray:
    data = np.asarray(message.data, dtype=np.float32)
    offset = int(message.layout.data_offset)
    if offset:
        data = data[offset:]
    dims = message.layout.dim

    if len(dims) >= 2:
        label0, label1 = dims[0].label, dims[1].label
        if label0 == "row_index" and label1 == "column_index":
            rows, cols = int(dims[0].size), int(dims[1].size)
            if rows * cols != data.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata")
            return data.reshape((rows, cols), order="C")
        if label0 == "column_index" and label1 == "row_index":
            cols, rows = int(dims[0].size), int(dims[1].size)
            if rows * cols != data.size:
                raise ValueError(f"Layer '{name}' has inconsistent layout metadata")
            return data.reshape((rows, cols), order="F")

    if dims:
        cols = int(dims[0].size or 1)
        rows = int(dims[1].size) if len(dims) > 1 else data.size // cols
    else:
        cols = int(math.sqrt(data.size)) if data.size else 0
        rows = cols
    if rows * cols != data.size:
        raise ValueError(f"Layer '{name}' has inconsistent layout metadata")
    return data.reshape((rows, cols), order="C")


def encode_layer(values: np.ndarray) -> Float32MultiArray:
    """Use the column-major layout expected by ROS grid_map."""

    array = np.asarray(values, dtype=np.float32)
    if array.ndim != 2:
        raise ValueError("GridMap layers must be 2-D")
    rows, cols = array.shape
    message = Float32MultiArray()
    message.layout = MultiArrayLayout()
    message.layout.dim.append(
        MultiArrayDimension(label="column_index", size=cols, stride=rows * cols)
    )
    message.layout.dim.append(
        MultiArrayDimension(label="row_index", size=rows, stride=rows)
    )
    contiguous = np.ascontiguousarray(array.T)
    message.data = float_array("f")
    message.data.frombytes(contiguous.tobytes())
    return message
