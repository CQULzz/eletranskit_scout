#!/usr/bin/env python3
"""Record persistent-global, local-crop, and ZT GridMaps into a GIF."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, Rectangle
import numpy as np
from PIL import Image
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from zt_traversability_ros2.gridmap_codec import decode_layer


GLOBAL_TOPIC = "/elevation_mapping_node/elevation_map_global"
LOCAL_TOPIC = "/elevation_mapping_node/elevation_map_global_local"
ZT_TOPIC = "/zt_traversability/global_local_8dir"


def stamp_ns(message: GridMap) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def layers(message: GridMap) -> dict[str, np.ndarray]:
    return {
        name: decode_layer(name, data)
        for name, data in zip(message.layers, message.data)
    }


def extent(message: GridMap, relative: bool = False) -> tuple[float, float, float, float]:
    half_x = float(message.info.length_x) / 2.0
    half_y = float(message.info.length_y) / 2.0
    if relative:
        return (-half_x, half_x, -half_y, half_y)
    center_x = float(message.info.pose.position.x)
    center_y = float(message.info.pose.position.y)
    return (
        center_x - half_x,
        center_x + half_x,
        center_y - half_y,
        center_y + half_y,
    )


def masked_cmap(name: str):
    result = plt.get_cmap(name).copy()
    result.set_bad("black")
    return result


def finite_plot_bounds(
    message: GridMap, values: np.ndarray, padding_m: float = 0.8
) -> tuple[float, float, float, float]:
    """Return tight world-coordinate bounds around a GridMap's finite cells."""
    world = extent(message)
    rows, columns = values.shape
    finite_rows, finite_columns = np.where(np.isfinite(values))
    if finite_rows.size == 0:
        return world
    cell_x = (world[1] - world[0]) / columns
    cell_y = (world[3] - world[2]) / rows
    return (
        world[0] + finite_columns.min() * cell_x - padding_m,
        world[0] + (finite_columns.max() + 1) * cell_x + padding_m,
        world[2] + finite_rows.min() * cell_y - padding_m,
        world[2] + (finite_rows.max() + 1) * cell_y + padding_m,
    )


def finite_median(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    return float(np.median(finite)) if finite.size else float("nan")


def encode_gif(
    frame_paths: list[Path], gif_path: Path, frame_duration_ms: int
) -> None:
    """Encode PNG frames with both old and new Pillow releases."""
    resampling = getattr(Image, "Resampling", Image)
    quantize = getattr(Image, "Quantize", Image)
    palette_frames: list[Image.Image] = []
    for path in frame_paths:
        with Image.open(path) as image:
            resized = image.convert("RGB")
            # Preserve enough vertical resolution for the large global panel,
            # crop rectangle, and the two stacked traversability panels.
            resized.thumbnail((1400, 720), resampling.LANCZOS)
            palette_frames.append(
                resized.quantize(colors=128, method=quantize.MEDIANCUT)
            )
    palette_frames[0].save(
        gif_path,
        save_all=True,
        append_images=palette_frames[1:],
        duration=frame_duration_ms,
        loop=0,
        optimize=False,
        disposal=2,
    )


class GlobalZtGifRecorder(Node):
    def __init__(
        self,
        output_dir: Path,
        frame_stride: int,
        gif_frame_ms: int,
    ) -> None:
        super().__init__("global_zt_gif_recorder")
        self.output_dir = output_dir.resolve()
        self.frames_dir = self.output_dir / "gif_frames"
        self.frames_dir.mkdir(parents=True, exist_ok=False)
        self.frame_stride = int(frame_stride)
        self.gif_frame_ms = int(gif_frame_ms)
        self.started = time.time()
        self.global_count = 0
        self.local_count = 0
        self.zt_count = 0
        self.saved_count = 0
        self.skipped_stride = 0
        self.skipped_missing_local = 0
        self.last_saved_stamp: int | None = None
        self.latest_global: GridMap | None = None
        self.local_by_stamp: dict[int, GridMap] = {}
        self.frame_paths: list[Path] = []
        self.robot_track: list[tuple[float, float]] = []
        self._finalized = False

        self._manifest_handle = (self.output_dir / "manifest.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._manifest = csv.DictWriter(
            self._manifest_handle,
            fieldnames=[
                "gif_frame_index",
                "source_zt_index",
                "stamp_ns",
                "global_revision_frame",
                "local_center_x_m",
                "local_center_y_m",
                "local_finite_cells",
                "zt_valid_cells",
                "worst_probability_median",
                "best_probability_median",
                "png_file",
            ],
        )
        self._manifest.writeheader()

        persistent_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        inference_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.create_subscription(GridMap, GLOBAL_TOPIC, self._global, persistent_qos),
            self.create_subscription(GridMap, LOCAL_TOPIC, self._local, persistent_qos),
            self.create_subscription(GridMap, ZT_TOPIC, self._zt, inference_qos),
        ]
        self.get_logger().info(
            f"Recording global/local/ZT frames under {self.output_dir}"
        )

    def _global(self, message: GridMap) -> None:
        self.global_count += 1
        self.latest_global = message

    def _local(self, message: GridMap) -> None:
        self.local_count += 1
        key = stamp_ns(message)
        self.local_by_stamp[key] = message
        if len(self.local_by_stamp) > 200:
            for stale in sorted(self.local_by_stamp)[:-100]:
                del self.local_by_stamp[stale]

    def _zt(self, message: GridMap) -> None:
        self.zt_count += 1
        key = stamp_ns(message)
        if key == self.last_saved_stamp:
            return
        local = self.local_by_stamp.pop(key, None)
        if local is None:
            self.skipped_missing_local += 1
            self.get_logger().warning(
                f"Skipping ZT stamp {key}: matching local global-map crop is absent",
                throttle_duration_sec=2.0,
            )
            return
        if self.zt_count % self.frame_stride != 0:
            self.skipped_stride += 1
            return
        try:
            self._save_frame(self.latest_global or local, local, message, key)
            self.last_saved_stamp = key
        except Exception as exc:
            self.get_logger().error(f"Failed to render stamp {key}: {exc}")

    def _save_frame(
        self, global_message: GridMap, local_message: GridMap, zt_message: GridMap, key: int
    ) -> None:
        global_layers = layers(global_message)
        local_layers = layers(local_message)
        zt_layers = layers(zt_message)
        global_height = global_layers["elevation"]
        local_height = local_layers["elevation"]
        probabilities = np.stack([zt_layers[f"zt_dir_{i}"] for i in range(8)])
        valid = zt_layers["zt_valid"] > 0.5
        worst = np.min(np.where(np.isfinite(probabilities), probabilities, 1.0), axis=0)
        best = np.max(np.where(np.isfinite(probabilities), probabilities, 0.0), axis=0)
        worst[~valid] = np.nan
        best[~valid] = np.nan

        figure = plt.figure(figsize=(16, 8.2), constrained_layout=True)
        grid = figure.add_gridspec(2, 4, width_ratios=(1.25, 1.25, 1.0, 1.0))
        global_axis = figure.add_subplot(grid[:, :2])
        local_axis = figure.add_subplot(grid[0, 2:])
        worst_axis = figure.add_subplot(grid[1, 2])
        best_axis = figure.add_subplot(grid[1, 3])
        axes = (global_axis, local_axis, worst_axis, best_axis)
        height_cmap = masked_cmap("terrain")
        probability_cmap = masked_cmap("RdYlGn")
        global_image = global_axis.imshow(
            np.ma.masked_invalid(global_height),
            origin="lower",
            extent=extent(global_message),
            cmap=height_cmap,
            vmin=-1.0,
            vmax=2.0,
        )
        robot_x = float(local_message.info.pose.position.x)
        robot_y = float(local_message.info.pose.position.y)
        self.robot_track.append((robot_x, robot_y))
        track = np.asarray(self.robot_track)
        global_axis.plot(
            track[:, 0], track[:, 1], color="cyan", linewidth=1.8, label="vehicle path"
        )
        global_axis.plot(
            robot_x,
            robot_y,
            marker="x",
            color="magenta",
            markeredgewidth=2.5,
            markersize=11,
            label="vehicle / sensor center",
        )
        crop_width = float(local_message.info.length_x)
        crop_height = float(local_message.info.length_y)
        crop_box = Rectangle(
            (robot_x - crop_width / 2.0, robot_y - crop_height / 2.0),
            crop_width,
            crop_height,
            fill=False,
            edgecolor="darkorange",
            linewidth=3.0,
            label=f"local crop {crop_width:.0f} x {crop_height:.0f} m",
        )
        global_axis.add_patch(crop_box)
        bounds = finite_plot_bounds(global_message, global_height)
        global_axis.set_xlim(
            min(bounds[0], robot_x - crop_width / 2.0 - 0.8),
            max(bounds[1], robot_x + crop_width / 2.0 + 0.8),
        )
        global_axis.set_ylim(
            min(bounds[2], robot_y - crop_height / 2.0 - 0.8),
            max(bounds[3], robot_y + crop_height / 2.0 + 0.8),
        )
        global_axis.set_title("1  Persistent GLOBAL elevation map", fontsize=15)
        global_axis.legend(loc="upper left", fontsize=9)
        figure.colorbar(global_image, ax=global_axis, fraction=0.035, pad=0.03, label="height [m]")

        local_image = local_axis.imshow(
            np.ma.masked_invalid(local_height),
            origin="lower",
            extent=extent(local_message, relative=True),
            cmap=height_cmap,
            vmin=-1.0,
            vmax=2.0,
        )
        local_axis.plot(
            0.0,
            0.0,
            marker="x",
            color="magenta",
            markeredgewidth=2.5,
            markersize=10,
        )
        local_axis.set_title("2  SENSOR-RANGE crop (orange box)", fontsize=14)
        figure.colorbar(local_image, ax=local_axis, fraction=0.046, pad=0.03, label="height [m]")
        figure.add_artist(
            ConnectionPatch(
                xyA=(robot_x + crop_width / 2.0, robot_y + crop_height / 2.0),
                coordsA=global_axis.transData,
                xyB=(0.0, 0.5),
                coordsB=local_axis.transAxes,
                arrowstyle="-|>",
                mutation_scale=18,
                color="darkorange",
                linewidth=2.5,
            )
        )

        worst_image = worst_axis.imshow(
            np.ma.masked_invalid(worst),
            origin="lower",
            extent=extent(local_message, relative=True),
            cmap=probability_cmap,
            vmin=0.0,
            vmax=1.0,
        )
        worst_axis.set_title("3  ZT worst of 8 dirs")
        figure.colorbar(worst_image, ax=worst_axis, fraction=0.046, pad=0.03)

        best_image = best_axis.imshow(
            np.ma.masked_invalid(best),
            origin="lower",
            extent=extent(local_message, relative=True),
            cmap=probability_cmap,
            vmin=0.0,
            vmax=1.0,
        )
        best_axis.set_title("4  ZT best direction")
        figure.colorbar(best_image, ax=best_axis, fraction=0.046, pad=0.03)

        for index, axis in enumerate(axes):
            axis.set_xlabel("map x [m]" if index == 0 else "local x [m]")
            axis.set_ylabel("map y [m]" if index == 0 else "local y [m]")
            axis.set_aspect("equal")
        figure.suptitle(
            "GLOBAL map  →  orange sensor crop  →  ZT 8-direction traversability\n"
            f"map revision {self.global_count} | vehicle=({robot_x:.2f}, {robot_y:.2f}) m",
            fontsize=17,
        )
        path = self.frames_dir / f"frame_{self.saved_count:06d}.png"
        figure.savefig(path, dpi=100)
        plt.close(figure)

        finite_local = np.isfinite(local_height)
        self._manifest.writerow(
            {
                "gif_frame_index": self.saved_count,
                "source_zt_index": self.zt_count,
                "stamp_ns": key,
                "global_revision_frame": self.global_count,
                "local_center_x_m": robot_x,
                "local_center_y_m": robot_y,
                "local_finite_cells": int(finite_local.sum()),
                "zt_valid_cells": int(valid.sum()),
                "worst_probability_median": finite_median(worst),
                "best_probability_median": finite_median(best),
                "png_file": str(path.relative_to(self.output_dir)),
            }
        )
        self._manifest_handle.flush()
        self.frame_paths.append(path)
        self.saved_count += 1
        if self.saved_count % 10 == 0:
            self.get_logger().info(f"Saved GIF source frames: {self.saved_count}")

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self._manifest_handle.flush()
        self._manifest_handle.close()
        gif_path = self.output_dir / "global_to_local_to_zt.gif"
        errors: list[str] = []
        if self.frame_paths:
            try:
                encode_gif(self.frame_paths, gif_path, self.gif_frame_ms)
            except Exception as exc:
                errors.append(f"GIF encoding failed: {exc}")
        else:
            errors.append("No synchronized ZT/local frames were recorded")

        summary = {
            "status": "passed" if not errors else "failed",
            "output_dir": str(self.output_dir),
            "elapsed_wall_seconds": time.time() - self.started,
            "received_global_frames": self.global_count,
            "received_local_frames": self.local_count,
            "received_zt_frames": self.zt_count,
            "saved_gif_frames": self.saved_count,
            "skipped_by_stride": self.skipped_stride,
            "skipped_missing_local": self.skipped_missing_local,
            "gif_frame_duration_ms": self.gif_frame_ms,
            "gif_file": str(gif_path) if gif_path.exists() else "",
            "errors": errors,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(json.dumps(summary, ensure_ascii=False))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--gif-frame-ms", type=int, default=100)
    parser.add_argument(
        "--encode-existing",
        action="store_true",
        help="only encode existing output_dir/gif_frames PNGs and repair summary.json",
    )
    # ros2 launch appends --ros-args/remaps to executable argv. Strip those
    # before parsing recorder-specific positional and optional arguments.
    arguments = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if arguments.frame_stride <= 0 or arguments.gif_frame_ms <= 0:
        parser.error("frame stride and GIF duration must be positive")
    return arguments


def main() -> None:
    arguments = parse_args()
    if arguments.encode_existing:
        frame_paths = sorted((arguments.output_dir / "gif_frames").glob("frame_*.png"))
        if not frame_paths:
            raise SystemExit(f"no PNG frames under {arguments.output_dir / 'gif_frames'}")
        gif_path = arguments.output_dir / "global_to_local_to_zt.gif"
        encode_gif(frame_paths, gif_path, arguments.gif_frame_ms)
        summary_path = arguments.output_dir / "summary.json"
        summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
        summary.update(
            {
                "status": "passed",
                "saved_gif_frames": len(frame_paths),
                "gif_frame_duration_ms": arguments.gif_frame_ms,
                "gif_file": str(gif_path),
                "errors": [],
            }
        )
        summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        return
    rclpy.init()
    node = GlobalZtGifRecorder(
        arguments.output_dir, arguments.frame_stride, arguments.gif_frame_ms
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
