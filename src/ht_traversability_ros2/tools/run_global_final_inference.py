#!/usr/bin/env python3
"""Run offline HT inference on the final view of persistent global maps."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sqlite3
import sys
import time
import zlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm
import numpy as np
import torch


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ht_traversability_ros2.core import (  # noqa: E402
    RECEPTIVE_FIELD,
    build_output_layers,
    nearest_fill_nan,
    valid_window_ratio,
)
from ht_traversability_ros2.model import load_student_model  # noqa: E402


BAG_NAMES = (
    "rosbag2_2026_08_20-17_17_04",
    "rosbag2_2026_08_20-17_20_44",
    "rosbag2_2026_08_20-17_21_56",
)
VIEW_LENGTH_M = 30.0
VIEW_CENTER_XY_M = (0.0, 0.0)
EXPECTED_RESOLUTION_M = 0.04
MIN_VALID_RATIO = 0.70
OUTPUT_TILE_CELLS = 200


def read_metadata(connection: sqlite3.Connection) -> dict[str, object]:
    return {
        key: json.loads(value)
        for key, value in connection.execute("SELECT key, value FROM metadata")
    }


def extract_window(database: Path) -> tuple[np.ndarray, dict[str, object]]:
    """Extract the same centered 30 m window used by the final global PNG."""

    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    try:
        metadata = read_metadata(connection)
        resolution = float(metadata["resolution"])
        if not np.isclose(resolution, EXPECTED_RESOLUTION_M, atol=1.0e-9):
            raise ValueError(f"Unexpected global-map resolution {resolution}")
        tile_size = int(metadata["tile_size_cells"])
        cells_x = int(round(VIEW_LENGTH_M / resolution))
        cells_y = cells_x
        if cells_x % 2 != 0 or cells_y % 2 != 0:
            raise ValueError("This extractor expects an even-sized global view")
        center_ix = int(round(VIEW_CENTER_XY_M[0] / resolution))
        center_iy = int(round(VIEW_CENTER_XY_M[1] / resolution))
        start_ix = center_ix - cells_x // 2
        start_iy = center_iy - cells_y // 2
        end_ix = start_ix + cells_x
        end_iy = start_iy + cells_y
        output = np.full((cells_y, cells_x), np.nan, dtype=np.float32)

        for tile_x, tile_y, payload in connection.execute(
            "SELECT tile_x, tile_y, payload FROM tiles WHERE level=0"
        ):
            tile_ix0 = int(tile_x) * tile_size
            tile_iy0 = int(tile_y) * tile_size
            overlap_ix0 = max(start_ix, tile_ix0)
            overlap_ix1 = min(end_ix, tile_ix0 + tile_size)
            overlap_iy0 = max(start_iy, tile_iy0)
            overlap_iy1 = min(end_iy, tile_iy0 + tile_size)
            if overlap_ix0 >= overlap_ix1 or overlap_iy0 >= overlap_iy1:
                continue
            raw = zlib.decompress(payload)
            expected_bytes = tile_size * tile_size * np.dtype("<f4").itemsize
            if len(raw) != expected_bytes:
                raise ValueError(
                    f"Corrupt tile ({tile_x}, {tile_y}): "
                    f"{len(raw)} bytes instead of {expected_bytes}"
                )
            tile = np.frombuffer(raw, dtype="<f4").reshape(tile_size, tile_size)
            source_y0 = overlap_iy0 - tile_iy0
            source_x0 = overlap_ix0 - tile_ix0
            destination_y0 = overlap_iy0 - start_iy
            destination_x0 = overlap_ix0 - start_ix
            height = overlap_iy1 - overlap_iy0
            width = overlap_ix1 - overlap_ix0
            output[
                destination_y0 : destination_y0 + height,
                destination_x0 : destination_x0 + width,
            ] = tile[
                source_y0 : source_y0 + height,
                source_x0 : source_x0 + width,
            ]
    finally:
        connection.close()
    return output, metadata


def final_manifest_row(bag_dir: Path) -> dict[str, str]:
    with (bag_dir / "global_manifest.csv").open(
        newline="", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"Empty global manifest under {bag_dir}")
    return rows[-1]


def infer_tiled(
    model: torch.nn.Module, height: np.ndarray, device: torch.device
) -> tuple[dict[str, np.ndarray], float]:
    """Infer exact valid-convolution output in bounded-memory spatial tiles."""

    filled, finite_mask = nearest_fill_nan(height)
    support_ratio = valid_window_ratio(finite_mask, RECEPTIVE_FIELD)
    valid_output = support_ratio >= MIN_VALID_RATIO
    output_rows, output_cols = valid_output.shape
    probabilities = np.empty((8, output_rows, output_cols), dtype=np.float32)
    context = RECEPTIVE_FIELD - 1

    started = time.perf_counter()
    with torch.inference_mode():
        for output_y0 in range(0, output_rows, OUTPUT_TILE_CELLS):
            output_y1 = min(output_y0 + OUTPUT_TILE_CELLS, output_rows)
            for output_x0 in range(0, output_cols, OUTPUT_TILE_CELLS):
                output_x1 = min(output_x0 + OUTPUT_TILE_CELLS, output_cols)
                input_block = filled[
                    output_y0 : output_y1 + context,
                    output_x0 : output_x1 + context,
                ]
                tensor = torch.from_numpy(np.ascontiguousarray(input_block))
                tensor = tensor.unsqueeze(0).unsqueeze(0).to(device=device)
                logits = model(tensor)
                expected_shape = (
                    1,
                    8,
                    2,
                    output_y1 - output_y0,
                    output_x1 - output_x0,
                )
                if tuple(logits.shape) != expected_shape:
                    raise RuntimeError(
                        f"Unexpected HT tile output {tuple(logits.shape)}, "
                        f"expected {expected_shape}"
                    )
                tile_probabilities = torch.softmax(logits, dim=2)[0, :, 1]
                probabilities[
                    :, output_y0:output_y1, output_x0:output_x1
                ] = tile_probabilities.cpu().numpy()
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elapsed_ms = (time.perf_counter() - started) * 1000.0

    layers = build_output_layers(probabilities, valid_output, height.shape)
    layers["ht_height"] = height.astype(np.float32, copy=False)
    direction_stack = np.stack([layers[f"ht_dir_{index}"] for index in range(8)])
    valid_full = layers["ht_valid"] > 0.5
    worst = np.min(
        np.where(np.isfinite(direction_stack), direction_stack, 1.0), axis=0
    ).astype(np.float32)
    worst[~valid_full] = np.nan
    layers["ht_worst_prob"] = worst
    return layers, elapsed_ms


def map_extent(height: np.ndarray) -> tuple[float, float, float, float]:
    length_x = height.shape[1] * EXPECTED_RESOLUTION_M
    length_y = height.shape[0] * EXPECTED_RESOLUTION_M
    return (-length_x / 2, length_x / 2, -length_y / 2, length_y / 2)


def masked_cmap(name: str):
    color_map = plt.get_cmap(name).copy()
    color_map.set_bad(color="black")
    return color_map


def save_result_plot(path: Path, layers: dict[str, np.ndarray], title: str) -> None:
    height = layers["ht_height"]
    extent = map_extent(height)
    figure, axes = plt.subplots(2, 2, figsize=(14, 12), constrained_layout=True)

    height_image = axes[0, 0].imshow(
        np.ma.masked_invalid(height),
        origin="lower",
        extent=extent,
        cmap=masked_cmap("terrain"),
        vmin=-1.0,
        vmax=2.0,
    )
    axes[0, 0].set_title("Input elevation [m]")
    figure.colorbar(height_image, ax=axes[0, 0])

    worst_image = axes[0, 1].imshow(
        np.ma.masked_invalid(layers["ht_worst_prob"]),
        origin="lower",
        extent=extent,
        cmap=masked_cmap("RdYlGn"),
        vmin=0.0,
        vmax=1.0,
    )
    axes[0, 1].set_title("Conservative traversability: min of 8 dirs")
    figure.colorbar(worst_image, ax=axes[0, 1])

    best_image = axes[1, 0].imshow(
        np.ma.masked_invalid(layers["ht_best_prob"]),
        origin="lower",
        extent=extent,
        cmap=masked_cmap("RdYlGn"),
        vmin=0.0,
        vmax=1.0,
    )
    axes[1, 0].set_title("Best-direction traversability probability")
    figure.colorbar(best_image, ax=axes[1, 0])

    direction_map = np.ma.masked_invalid(layers["ht_best_dir"])
    direction_cmap = masked_cmap("turbo")
    direction_norm = BoundaryNorm(np.arange(-0.5, 8.5, 1.0), direction_cmap.N)
    direction_image = axes[1, 1].imshow(
        direction_map,
        origin="lower",
        extent=extent,
        cmap=direction_cmap,
        norm=direction_norm,
    )
    axes[1, 1].set_title("Best raw HT direction channel")
    direction_bar = figure.colorbar(
        direction_image, ax=axes[1, 1], ticks=np.arange(8)
    )
    direction_bar.set_label("channel index (not vehicle-direction calibrated)")

    for axis in axes.flat:
        axis.set_xlabel("map x [m]")
        axis.set_ylabel("map y [m]")
        axis.set_aspect("equal")
    figure.suptitle(title)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def save_direction_plot(path: Path, layers: dict[str, np.ndarray], title: str) -> None:
    extent = map_extent(layers["ht_height"])
    figure, axes = plt.subplots(2, 4, figsize=(19, 9), constrained_layout=True)
    image = None
    for direction, axis in enumerate(axes.flat):
        image = axis.imshow(
            np.ma.masked_invalid(layers[f"ht_dir_{direction}"]),
            origin="lower",
            extent=extent,
            cmap=masked_cmap("RdYlGn"),
            vmin=0.0,
            vmax=1.0,
        )
        axis.set_title(f"ht_dir_{direction}")
        axis.set_xlabel("map x [m]")
        axis.set_ylabel("map y [m]")
        axis.set_aspect("equal")
    figure.colorbar(image, ax=axes, shrink=0.85, label="traversability probability")
    figure.suptitle(title)
    figure.savefig(path, dpi=130)
    plt.close(figure)


def finite_stats(values: np.ndarray) -> dict[str, float | int]:
    finite = np.asarray(values)[np.isfinite(values)]
    if not finite.size:
        return {"count": 0}
    return {
        "count": int(finite.size),
        "min": float(np.min(finite)),
        "p05": float(np.quantile(finite, 0.05)),
        "median": float(np.median(finite)),
        "p95": float(np.quantile(finite, 0.95)),
        "max": float(np.max(finite)),
    }


def run_bag(
    bag_name: str,
    source_root: Path,
    output_root: Path,
    model: torch.nn.Module,
    device: torch.device,
    weight_path: Path,
) -> dict[str, object]:
    source_dir = source_root / bag_name
    output_dir = output_root / bag_name
    output_dir.mkdir(parents=True, exist_ok=False)
    final_row = final_manifest_row(source_dir)
    height, metadata = extract_window(source_dir / "global_tiles.sqlite3")
    expected_finite = int(final_row["finite_cells"])
    actual_finite = int(np.isfinite(height).sum())
    if actual_finite != expected_finite:
        raise RuntimeError(
            f"{bag_name}: extracted finite cells {actual_finite} != "
            f"final manifest {expected_finite}"
        )

    layers, elapsed_ms = infer_tiled(model, height, device)
    valid = layers["ht_valid"] > 0.5
    result_name = f"frame_{int(final_row['frame_index']):06d}_ht_result.png"
    directions_name = f"frame_{int(final_row['frame_index']):06d}_ht_8dirs.png"
    arrays_name = f"frame_{int(final_row['frame_index']):06d}_ht_layers.npz"
    title = f"{bag_name} final global frame / HT 8-direction inference"
    save_result_plot(output_dir / result_name, layers, title)
    save_direction_plot(output_dir / directions_name, layers, title)
    np.savez_compressed(
        output_dir / arrays_name,
        **{name: value.astype(np.float32) for name, value in layers.items()},
    )

    summary: dict[str, object] = {
        "bag": bag_name,
        "source_database": str(source_dir / "global_tiles.sqlite3"),
        "source_final_png": str(source_dir / final_row["png_file"]),
        "source_frame_index": int(final_row["frame_index"]),
        "source_global_revision": int(final_row["global_revision"]),
        "source_stamp_ns": int(final_row["stamp_ns"]),
        "model_weight": str(weight_path),
        "device": str(device),
        "resolution_m": float(metadata["resolution"]),
        "input_shape": list(height.shape),
        "model_output_shape": [height.shape[0] - 50, height.shape[1] - 50],
        "receptive_field": RECEPTIVE_FIELD,
        "minimum_observed_ratio_per_window": MIN_VALID_RATIO,
        "inference_tiling_output_cells": OUTPUT_TILE_CELLS,
        "inference_elapsed_ms": elapsed_ms,
        "input_finite_cells": actual_finite,
        "input_finite_fraction": float(actual_finite / height.size),
        "valid_output_cells": int(valid.sum()),
        "valid_output_fraction_of_full_map": float(valid.mean()),
        "probability_statistics": {
            **{
                f"ht_dir_{direction}": finite_stats(layers[f"ht_dir_{direction}"])
                for direction in range(8)
            },
            "ht_worst_prob": finite_stats(layers["ht_worst_prob"]),
            "ht_best_prob": finite_stats(layers["ht_best_prob"]),
        },
        "result_image": str(output_dir / result_name),
        "directions_image": str(output_dir / directions_name),
        "raw_layers_npz": str(output_dir / arrays_name),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return summary


def write_report(output_root: Path, summaries: list[dict[str, object]]) -> None:
    lines = [
        "# 三个最终全局高程图的 HT 通过性推理报告",
        "",
        "## 推理设置",
        "",
        f"- 模型：HT Scout eight-direction student。",
        f"- 输入：每个全局数据库最后 revision 的 30 m × 30 m 原始高程窗口。",
        f"- 分辨率：{EXPECTED_RESOLUTION_M:.2f} m/cell；输入 750×750。",
        f"- 感受野：{RECEPTIVE_FIELD}×{RECEPTIVE_FIELD}；模型有效输出 700×700。",
        f"- 有效性门限：每个感受野中至少 {MIN_VALID_RATIO:.0%} 为真实有限高程。",
        "- 推理采用带完整上下文的空间分块，仅用于控制显存；不改变模型计算结果。",
        "- `ht_worst_prob` 是八方向最小概率，用于保守查看；`ht_best_prob` 是八方向最大概率。",
        "- 八个原始通道尚未标定到车辆的前、后、左、右方向。",
        "",
        "## 结果",
        "",
        "| 数据包 | 最后一帧 | 有效输出格点 | 有效比例 | 推理耗时 | 保守概率中位数 | 最佳概率中位数 |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for summary in summaries:
        statistics = summary["probability_statistics"]
        lines.append(
            "| `{bag}` | {frame} | {valid} | {ratio:.2%} | {elapsed:.1f} ms | "
            "{worst:.4f} | {best:.4f} |".format(
                bag=summary["bag"],
                frame=summary["source_frame_index"],
                valid=summary["valid_output_cells"],
                ratio=summary["valid_output_fraction_of_full_map"],
                elapsed=summary["inference_elapsed_ms"],
                worst=statistics["ht_worst_prob"]["median"],
                best=statistics["ht_best_prob"]["median"],
            )
        )
    lines.extend(
        [
            "",
            "## 图片说明",
            "",
            "每个数据包目录中：",
            "",
            "- `*_ht_result.png`：输入高程、八方向最小概率、最佳方向概率和最佳原始方向通道。",
            "- `*_ht_8dirs.png`：八个方向概率的 2×4 总览。",
            "- `*_ht_layers.npz`：高程、有效掩码、八方向概率、最小/最大概率和最佳通道原始数组。",
            "- 黑色区域表示未知或未达到 70% 有效观测门限，不代表概率为零。",
            "",
        ]
    )
    (output_root / "README.md").write_text("\n".join(lines), encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        type=Path,
        required=True,
        help="Directory containing the saved global-elevation frame sets",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        required=True,
        help="New directory in which to write HT inference results",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=PACKAGE_ROOT / "models" / "student_best_epoch_weights.pth",
    )
    parser.add_argument("--device", default="cuda")
    return parser.parse_args()


def main() -> None:
    arguments = parse_args()
    source_root = arguments.source_root.resolve()
    output_root = arguments.output_root.resolve()
    weight_path = arguments.model.resolve()
    if output_root.exists() and any(output_root.iterdir()):
        raise FileExistsError(f"Refusing to overwrite non-empty {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    if not weight_path.is_file():
        raise FileNotFoundError(weight_path)
    device = torch.device(arguments.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but torch.cuda.is_available() is false")

    model = load_student_model(str(weight_path), device)
    model.eval()
    summaries = [
        run_bag(name, source_root, output_root, model, device, weight_path)
        for name in BAG_NAMES
    ]
    report = {
        "status": "passed",
        "source_root": str(source_root),
        "output_root": str(output_root),
        "model_weight": str(weight_path),
        "device": str(device),
        "bags": summaries,
    }
    (output_root / "summary.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    write_report(output_root, summaries)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
