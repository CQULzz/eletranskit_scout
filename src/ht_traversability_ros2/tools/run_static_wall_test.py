#!/usr/bin/env python3
"""Publish synthetic wall GridMaps, collect HT outputs, and save a report."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node

from ht_traversability_ros2.gridmap_codec import decode_layer, encode_layer


GRID_SIZE = 250
RESOLUTION_M = 0.04
MAP_LENGTH_M = GRID_SIZE * RESOLUTION_M
HEIGHTS_CM = tuple(range(1, 51, 2))
WALL_LENGTH_M = 3.0
WALL_THICKNESS_M = 0.20
OUTPUT_LAYER_NAMES = tuple(
    [f"ht_dir_{index}" for index in range(8)]
    + ["ht_best_dir", "ht_best_prob", "ht_valid", "ht_height"]
)


def centered_bounds(center: int, count: int) -> tuple[int, int]:
    start = center - count // 2
    return start, start + count


def make_isolated_wall(height_cm: int) -> tuple[np.ndarray, tuple[slice, slice]]:
    """Return flat ground with one 3 m by 0.2 m raised strip."""

    height = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    center = GRID_SIZE // 2
    length_cells = round(WALL_LENGTH_M / RESOLUTION_M)
    thickness_cells = round(WALL_THICKNESS_M / RESOLUTION_M)
    row_start, row_end = centered_bounds(center, length_cells)
    col_start, col_end = centered_bounds(center, thickness_cells)
    height[row_start:row_end, col_start:col_end] = height_cm / 100.0

    # Evaluate the wall and a one-metre approach band on both sides, while
    # excluding segment endpoints where a vehicle could simply go around it.
    approach_cells = round(1.0 / RESOLUTION_M)
    roi = (
        slice(row_start + approach_cells // 2, row_end - approach_cells // 2),
        slice(col_start - approach_cells, col_end + approach_cells),
    )
    return height, roi


def make_overview_map() -> tuple[np.ndarray, list[dict[str, float]]]:
    """Arrange the 25 requested wall heights in a 5x5 overview."""

    height = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    centers = np.linspace(30, GRID_SIZE - 30, 5).round().astype(int)
    segment_length = round(1.0 / RESOLUTION_M)
    segment_thickness = round(0.16 / RESOLUTION_M)
    labels: list[dict[str, float]] = []
    for height_cm, (row, col) in zip(
        HEIGHTS_CM, ((row, col) for row in centers for col in centers)
    ):
        row_start, row_end = centered_bounds(int(row), segment_length)
        col_start, col_end = centered_bounds(int(col), segment_thickness)
        height[row_start:row_end, col_start:col_end] = height_cm / 100.0
        labels.append(
            {
                "height_cm": int(height_cm),
                "x_m": (float(col) - GRID_SIZE / 2.0) * RESOLUTION_M,
                "y_m": (float(row) - GRID_SIZE / 2.0) * RESOLUTION_M,
            }
        )
    return height, labels


def make_message(node: Node, height: np.ndarray) -> GridMap:
    message = GridMap()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = "odom"
    message.info.resolution = RESOLUTION_M
    message.info.length_x = MAP_LENGTH_M
    message.info.length_y = MAP_LENGTH_M
    message.info.pose.orientation.w = 1.0
    message.layers = ["inpaint"]
    message.basic_layers = []
    message.data = [encode_layer(height)]
    return message


def stamp_key(message: GridMap) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


def decode_output(message: GridMap) -> dict[str, np.ndarray]:
    if tuple(message.layers) != OUTPUT_LAYER_NAMES:
        raise RuntimeError(f"Unexpected HT output layers: {list(message.layers)}")
    layers = {
        name: decode_layer(name, data)
        for name, data in zip(message.layers, message.data)
    }
    for name, layer in layers.items():
        if layer.shape != (GRID_SIZE, GRID_SIZE):
            raise RuntimeError(f"Layer {name} has shape {layer.shape}")
    return layers


def wait_for_subscription(
    node: Node, publisher, expected_count: int, timeout_sec: float = 15.0
) -> None:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() >= expected_count:
            return
    raise RuntimeError(
        "Synthetic input topic has only "
        f"{publisher.get_subscription_count()} subscriber(s); expected {expected_count}"
    )


def publish_and_wait(
    node: Node,
    publisher,
    received: dict[tuple[int, int], GridMap],
    height: np.ndarray,
    timeout_sec: float = 10.0,
) -> tuple[GridMap, float]:
    message = make_message(node, height)
    key = stamp_key(message)
    started = time.perf_counter()
    publisher.publish(message)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        output = received.pop(key, None)
        if output is not None:
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            # The HT node is configured at 5 Hz; respect the throttle before
            # publishing the next unique test case.
            remaining = 0.22 - (time.perf_counter() - started)
            if remaining > 0.0:
                time.sleep(remaining)
            return output, elapsed_ms
    raise RuntimeError(f"Timed out waiting for HT output stamp={key}")


def probability_maps(layers: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    probabilities = np.stack([layers[f"ht_dir_{index}"] for index in range(8)])
    valid = layers["ht_valid"] > 0.5
    worst = np.min(np.where(np.isfinite(probabilities), probabilities, 1.0), axis=0)
    worst[~valid] = np.nan
    return probabilities, worst


def save_case_plot(
    path: Path,
    height: np.ndarray,
    worst: np.ndarray,
    best: np.ndarray,
    title: str,
) -> None:
    extent = (-MAP_LENGTH_M / 2, MAP_LENGTH_M / 2, -MAP_LENGTH_M / 2, MAP_LENGTH_M / 2)
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    input_image = axes[0].imshow(
        height,
        origin="lower",
        extent=extent,
        vmin=0.0,
        vmax=0.50,
        cmap="viridis",
    )
    axes[0].set_title("Input elevation [m]")
    figure.colorbar(input_image, ax=axes[0])
    worst_image = axes[1].imshow(
        worst, origin="lower", extent=extent, vmin=0.0, vmax=1.0, cmap="RdYlGn"
    )
    axes[1].set_title("Min probability over 8 dirs")
    figure.colorbar(worst_image, ax=axes[1])
    best_image = axes[2].imshow(
        best, origin="lower", extent=extent, vmin=0.0, vmax=1.0, cmap="RdYlGn"
    )
    axes[2].set_title("HT best probability")
    figure.colorbar(best_image, ax=axes[2])
    for axis in axes:
        axis.set_xlabel("map column [m]")
        axis.set_ylabel("map row [m]")
    figure.suptitle(title)
    figure.savefig(path, dpi=140)
    plt.close(figure)


def save_overview_input(
    path: Path, height: np.ndarray, labels: list[dict[str, float]]
) -> None:
    extent = (-MAP_LENGTH_M / 2, MAP_LENGTH_M / 2, -MAP_LENGTH_M / 2, MAP_LENGTH_M / 2)
    figure, axis = plt.subplots(figsize=(8, 7), constrained_layout=True)
    image = axis.imshow(
        height,
        origin="lower",
        extent=extent,
        vmin=0.0,
        vmax=0.50,
        cmap="viridis",
    )
    for label in labels:
        axis.text(
            label["x_m"],
            label["y_m"],
            f'{label["height_cm"]}cm',
            ha="center",
            va="center",
            fontsize=7,
            color="white" if label["height_cm"] > 15 else "black",
        )
    axis.set_title("Static wall overview: 1, 3, ..., 49 cm")
    axis.set_xlabel("map column [m]")
    axis.set_ylabel("map row [m]")
    figure.colorbar(image, ax=axis, label="elevation [m]")
    figure.savefig(path, dpi=160)
    plt.close(figure)


def save_summary_plot(path: Path, rows: list[dict[str, object]]) -> None:
    heights = np.asarray([row["height_cm"] for row in rows], dtype=float)
    direction_p05 = np.asarray([row["direction_p05"] for row in rows], dtype=float)
    worst_p05 = np.asarray([row["worst_p05"] for row in rows], dtype=float)
    worst_median = np.asarray([row["worst_median"] for row in rows], dtype=float)

    figure, axes = plt.subplots(2, 1, figsize=(11, 9), constrained_layout=True)
    for direction in range(8):
        axes[0].plot(
            heights,
            direction_p05[:, direction],
            marker="o",
            markersize=3,
            label=f"dir_{direction}",
        )
    axes[0].set_ylim(-0.02, 1.02)
    axes[0].set_ylabel("ROI probability p05")
    axes[0].set_title("Directional response around isolated walls")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncol=4)

    axes[1].plot(heights, worst_p05, marker="o", label="min-dir probability p05")
    axes[1].plot(heights, worst_median, marker="s", label="min-dir probability median")
    axes[1].set_ylim(-0.02, 1.02)
    axes[1].set_xlabel("wall height [cm]")
    axes[1].set_ylabel("traversability probability")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def serialize_layers(path: Path, height: np.ndarray, layers: dict[str, np.ndarray]) -> None:
    payload = {"input_height": height.astype(np.float32)}
    payload.update({name: value.astype(np.float32) for name, value in layers.items()})
    np.savez_compressed(path, **payload)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fieldnames = [
        "height_cm",
        "inference_roundtrip_ms",
        "valid_fraction",
        "worst_min",
        "worst_p05",
        "worst_median",
    ] + [f"dir_{index}_p05" for index in range(8)]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in fieldnames if key in row}
            for index, value in enumerate(row["direction_p05"]):
                flat[f"dir_{index}_p05"] = value
            writer.writerow(flat)


def write_report(path: Path, rows: list[dict[str, object]], bag_path: str) -> None:
    latency = np.asarray([row["inference_roundtrip_ms"] for row in rows])
    lowest = min(rows, key=lambda item: float(item["worst_p05"]))

    def first_height_below(threshold: float) -> str:
        for row in rows:
            if float(row["worst_p05"]) < threshold:
                return f"{int(row['height_cm'])} cm"
        return "未达到"

    lines = [
        "# HT 静态墙体高程图推理报告",
        "",
        "## 测试设置",
        "",
        f"- 高度序列：`{', '.join(str(value) for value in HEIGHTS_CM)} cm`。",
        "- 说明：从 1 cm 开始每隔 2 cm，最后一个值为 49 cm；该等差序列不会落到 50 cm。",
        f"- 地图：`{GRID_SIZE}×{GRID_SIZE}`，`{RESOLUTION_M:.2f} m/cell`，范围 `{MAP_LENGTH_M:.1f}×{MAP_LENGTH_M:.1f} m`。",
        f"- 隔离墙体：长 `{WALL_LENGTH_M:.2f} m`、厚 `{WALL_THICKNESS_M:.2f} m`，地面高度 0。",
        "- 墙体是 2.5D 高程图中的抬高条带，不包含垂直立面的多回波几何。",
        "- 方向层仍为模型原始 `ht_dir_0...7`，未标定到车辆物理方向。",
        f"- ROS bag：`{bag_path}`。",
        "",
        "## 执行摘要",
        "",
        f"- 完成隔离墙推理：{len(rows)} 张。",
        f"- ROS 发布到结果接收平均耗时：{latency.mean():.1f} ms。",
        f"- P95 耗时：{np.percentile(latency, 95):.1f} ms。",
        f"- ROI 最低 p05 出现在 {int(lowest['height_cm'])} cm，概率 {float(lowest['worst_p05']):.6f}。",
        f"- 最小方向 P05 首次低于 0.5：{first_height_below(0.5)}。",
        f"- 最小方向 P05 首次低于 0.1：{first_height_below(0.1)}。",
        f"- 最小方向 P05 首次低于 0.05：{first_height_below(0.05)}。",
        "",
        "## 每个高度结果",
        "",
        "| 高度(cm) | 往返耗时(ms) | 有效比例 | 八方向最小概率 | 最小方向P05 | 最小方向中位数 |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            "| {height_cm} | {inference_roundtrip_ms:.1f} | {valid_fraction:.4f} | "
            "{worst_min:.6f} | {worst_p05:.6f} | {worst_median:.6f} |".format(**row)
        )
    lines.extend(
        [
            "",
            "## 文件说明",
            "",
            "- `overview_input.png`：25 道不同高度墙的总览高程图。",
            "- `overview_result.png`：总览图 HT 结果。",
            "- `height_response.png`：墙高与八方向响应曲线。",
            "- `cases/*.npz`：每个高度的输入和所有 HT 输出层。",
            "- `cases/*.png`：每个隔离墙的输入、最小方向概率和 best probability。",
            "- `summary.csv/json`：机器可读统计。",
            "",
            "## 播放和查看",
            "",
            "只播放已保存的输入和 HT 输出，不要同时启动 HT 推理节点：",
            "",
            "```bash",
            "source /opt/ros/humble/setup.bash",
            f"ros2 bag play {bag_path} --loop --rate 0.5",
            "```",
            "",
            "另一个终端启动 RViz 预配置：",
            "",
            "```bash",
            "source /opt/ros/humble/setup.bash",
            "source install/local_setup.bash",
            "ros2 launch ht_traversability_ros2 ht_traversability.launch.py rviz:=true",
            "```",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--input-topic", default="/elevation_mapping_node/elevation_map_filter"
    )
    parser.add_argument(
        "--output-topic", default="/ht_traversability/traversability_8dir"
    )
    parser.add_argument("--bag-path", default="not recorded")
    parser.add_argument(
        "--expected-input-subscribers",
        type=int,
        default=1,
        help="Use 2 when both the HT node and rosbag recorder must connect",
    )
    args = parser.parse_args()

    output_dir = args.output_dir.expanduser().resolve()
    cases_dir = output_dir / "cases"
    cases_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node("ht_static_wall_test")
    publisher = node.create_publisher(GridMap, args.input_topic, 10)
    received: dict[tuple[int, int], GridMap] = {}

    def output_callback(message: GridMap) -> None:
        received[stamp_key(message)] = message

    subscription = node.create_subscription(
        GridMap, args.output_topic, output_callback, 10
    )
    del subscription  # The node retains ownership; silence static analyzers.

    wait_for_subscription(node, publisher, args.expected_input_subscribers)
    rows: list[dict[str, object]] = []
    manifest: list[dict[str, object]] = []
    for index, height_cm in enumerate(HEIGHTS_CM, start=1):
        height, roi = make_isolated_wall(height_cm)
        output, elapsed_ms = publish_and_wait(node, publisher, received, height)
        layers = decode_output(output)
        probabilities, worst = probability_maps(layers)
        roi_probabilities = probabilities[:, roi[0], roi[1]]
        roi_worst = worst[roi]
        finite_roi_worst = roi_worst[np.isfinite(roi_worst)]
        if finite_roi_worst.size == 0:
            raise RuntimeError(f"No valid HT output in wall ROI for {height_cm} cm")
        direction_p05 = [
            float(np.nanpercentile(roi_probabilities[direction], 5))
            for direction in range(8)
        ]
        row: dict[str, object] = {
            "height_cm": int(height_cm),
            "inference_roundtrip_ms": float(elapsed_ms),
            "valid_fraction": float((layers["ht_valid"] > 0.5).mean()),
            "worst_min": float(np.min(finite_roi_worst)),
            "worst_p05": float(np.percentile(finite_roi_worst, 5)),
            "worst_median": float(np.median(finite_roi_worst)),
            "direction_p05": direction_p05,
        }
        rows.append(row)
        stem = f"wall_{height_cm:02d}cm"
        serialize_layers(cases_dir / f"{stem}.npz", height, layers)
        save_case_plot(
            cases_dir / f"{stem}.png",
            height,
            worst,
            layers["ht_best_prob"],
            f"HT isolated wall test: {height_cm} cm",
        )
        manifest.append(
            {
                "case": stem,
                "height_cm": int(height_cm),
                "stamp": {"sec": int(output.header.stamp.sec), "nanosec": int(output.header.stamp.nanosec)},
            }
        )
        print(
            f"[{index:02d}/{len(HEIGHTS_CM)}] {height_cm:02d} cm: "
            f"roundtrip={elapsed_ms:.1f} ms, worst_p05={row['worst_p05']:.6f}"
        )

    overview, overview_labels = make_overview_map()
    overview_output, overview_elapsed_ms = publish_and_wait(
        node, publisher, received, overview
    )
    overview_layers = decode_output(overview_output)
    _, overview_worst = probability_maps(overview_layers)
    np.savez_compressed(output_dir / "overview_input.npz", elevation=overview)
    serialize_layers(output_dir / "overview_result.npz", overview, overview_layers)
    save_overview_input(output_dir / "overview_input.png", overview, overview_labels)
    save_case_plot(
        output_dir / "overview_result.png",
        overview,
        overview_worst,
        overview_layers["ht_best_prob"],
        "HT overview: 25 static wall heights",
    )
    save_summary_plot(output_dir / "height_response.png", rows)
    write_csv(output_dir / "summary.csv", rows)
    summary = {
        "height_sequence_cm": list(HEIGHTS_CM),
        "resolution_m": RESOLUTION_M,
        "grid_size": [GRID_SIZE, GRID_SIZE],
        "map_length_m": MAP_LENGTH_M,
        "isolated_wall_length_m": WALL_LENGTH_M,
        "isolated_wall_thickness_m": WALL_THICKNESS_M,
        "overview_roundtrip_ms": float(overview_elapsed_ms),
        "cases": rows,
        "manifest": manifest,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    write_report(output_dir / "report.md", rows, args.bag_path)

    node.destroy_node()
    rclpy.shutdown()
    print(f"PASS: saved static wall results to {output_dir}")


if __name__ == "__main__":
    main()
