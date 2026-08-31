#!/usr/bin/env python3
"""Record persistent-global, local-crop, and HT GridMaps into a GIF."""

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

from ht_traversability_ros2.gridmap_codec import decode_layer


GLOBAL_TOPIC = "/elevation_mapping_node/elevation_map_global"
LOCAL_TOPIC = "/elevation_mapping_node/elevation_map_global_local"
HT_TOPIC = "/ht_traversability/global_local"
CONSERVATIVE_THRESHOLD = 0.5


def stamp_ns(message: GridMap) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def layers(message: GridMap) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for name, data in zip(message.layers, message.data):
        grid_map_matrix = decode_layer(name, data)
        # GridMap matrices use row=-X,col=-Y. Convert them back to the natural
        # image/world convention row=+Y,col=+X before imshow, cropping, and
        # global/local alignment checks.
        result[name] = np.ascontiguousarray(
            np.flip(grid_map_matrix, axis=(0, 1)).T
        )
    return result


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


def compare_global_local(
    global_message: GridMap,
    global_height: np.ndarray,
    local_message: GridMap,
    local_height: np.ndarray,
) -> tuple[int, float, float]:
    """Compare the published local map against its world-coordinate global crop."""
    resolution = float(global_message.info.resolution)
    if not np.isclose(resolution, float(local_message.info.resolution)):
        return 0, float("nan"), float("nan")
    global_extent = extent(global_message)
    local_extent = extent(local_message)
    offset_x = int(round((local_extent[0] - global_extent[0]) / resolution))
    offset_y = int(round((local_extent[2] - global_extent[2]) / resolution))
    rows, columns = local_height.shape
    if (
        offset_x < 0
        or offset_y < 0
        or offset_x + columns > global_height.shape[1]
        or offset_y + rows > global_height.shape[0]
    ):
        return 0, float("nan"), float("nan")
    crop = global_height[
        offset_y : offset_y + rows,
        offset_x : offset_x + columns,
    ]
    global_finite = np.isfinite(crop)
    local_finite = np.isfinite(local_height)
    common = global_finite & local_finite
    mask_agreement = float((global_finite == local_finite).mean())
    max_error = (
        float(np.max(np.abs(crop[common] - local_height[common])))
        if common.any()
        else float("nan")
    )
    return int(common.sum()), mask_agreement, max_error


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


class GlobalHtGifRecorder(Node):
    def __init__(
        self,
        output_dir: Path,
        frame_stride: int,
        gif_frame_ms: int,
    ) -> None:
        super().__init__("global_ht_gif_recorder")
        self.output_dir = output_dir.resolve()
        self.frames_dir = self.output_dir / "gif_frames"
        self.frames_dir.mkdir(parents=True, exist_ok=False)
        self.frame_stride = int(frame_stride)
        self.gif_frame_ms = int(gif_frame_ms)
        self.started = time.time()
        self.global_count = 0
        self.local_count = 0
        self.ht_count = 0
        self.saved_count = 0
        self.skipped_stride = 0
        self.skipped_missing_local = 0
        self.last_saved_stamp: int | None = None
        self.latest_global: GridMap | None = None
        self.local_by_stamp: dict[int, GridMap] = {}
        self.frame_paths: list[Path] = []
        self.robot_track: list[tuple[float, float]] = []
        self.alignment_results: list[tuple[float, float]] = []
        self._finalized = False

        self._manifest_handle = (self.output_dir / "manifest.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._manifest = csv.DictWriter(
            self._manifest_handle,
            fieldnames=[
                "gif_frame_index",
                "source_ht_index",
                "stamp_ns",
                "global_revision_frame",
                "local_center_x_m",
                "local_center_y_m",
                "local_finite_cells",
                "ht_valid_cells",
                "global_local_common_cells",
                "global_local_mask_agreement",
                "global_local_max_abs_error_m",
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
            self.create_subscription(GridMap, HT_TOPIC, self._ht, inference_qos),
        ]
        self.get_logger().info(
            f"Recording global/local/HT frames under {self.output_dir}"
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

    def _ht(self, message: GridMap) -> None:
        self.ht_count += 1
        key = stamp_ns(message)
        if key == self.last_saved_stamp:
            return
        local = self.local_by_stamp.pop(key, None)
        if local is None:
            self.skipped_missing_local += 1
            self.get_logger().warning(
                f"Skipping HT stamp {key}: matching local global-map crop is absent",
                throttle_duration_sec=2.0,
            )
            return
        if self.ht_count % self.frame_stride != 0:
            self.skipped_stride += 1
            return
        try:
            self._save_frame(self.latest_global or local, local, message, key)
            self.last_saved_stamp = key
        except Exception as exc:
            self.get_logger().error(f"Failed to render stamp {key}: {exc}")

    def _save_frame(
        self, global_message: GridMap, local_message: GridMap, ht_message: GridMap, key: int
    ) -> None:
        global_layers = layers(global_message)
        local_layers = layers(local_message)
        ht_layers = layers(ht_message)
        global_height = global_layers["elevation"]
        local_height = local_layers["elevation"]
        aggregate_mode = "ht_traversable" in ht_layers
        if aggregate_mode:
            worst = ht_layers["ht_traversable"]
            valid = np.isfinite(worst)
            best = np.where(valid, 1.0 - worst, np.nan).astype(np.float32)
            left_result = worst
            right_result = best
        else:
            probabilities = np.stack([ht_layers[f"ht_dir_{i}"] for i in range(8)])
            valid = ht_layers["ht_valid"] > 0.5
            worst = np.min(
                np.where(np.isfinite(probabilities), probabilities, 1.0), axis=0
            )
            best = np.max(
                np.where(np.isfinite(probabilities), probabilities, 0.0), axis=0
            )
            worst[~valid] = np.nan
            best[~valid] = np.nan
            # Compare the old RViz choice (best of eight directions) against
            # the new conservative decision using the exact same inference.
            left_result = best
            right_result = np.where(
                valid,
                (worst >= CONSERVATIVE_THRESHOLD).astype(np.float32),
                np.nan,
            ).astype(np.float32)

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
        connection = ConnectionPatch(
            xyA=(robot_x + crop_width / 2.0, robot_y + crop_height / 2.0),
            coordsA=global_axis.transData,
            xyB=(0.0, 0.5),
            coordsB=local_axis.transAxes,
            arrowstyle="-|>",
            mutation_scale=18,
            color="darkorange",
            linewidth=2.5,
        )
        connection.set_in_layout(False)
        figure.add_artist(connection)

        worst_image = worst_axis.imshow(
            np.ma.masked_invalid(left_result),
            origin="lower",
            extent=extent(local_message, relative=True),
            cmap=probability_cmap,
            vmin=0.0,
            vmax=1.0,
        )
        worst_axis.set_title(
            "3  HT overall traversability"
            if aggregate_mode
            else "3  OLD: best of 8 direction probabilities"
        )
        figure.colorbar(worst_image, ax=worst_axis, fraction=0.046, pad=0.03)

        best_image = best_axis.imshow(
            np.ma.masked_invalid(right_result),
            origin="lower",
            extent=extent(local_message, relative=True),
            cmap=probability_cmap,
            vmin=0.0,
            vmax=1.0,
        )
        best_axis.set_title(
            "4  HT non-traversable mask"
            if aggregate_mode
            else f"4  NEW: all 8 >= {CONSERVATIVE_THRESHOLD:.2f} (binary)"
        )
        figure.colorbar(best_image, ax=best_axis, fraction=0.046, pad=0.03)

        for index, axis in enumerate(axes):
            axis.set_xlabel("map x [m]" if index == 0 else "local x [m]")
            axis.set_ylabel("map y [m]" if index == 0 else "local y [m]")
            axis.set_aspect("equal")
        figure.suptitle(
            "GLOBAL map  →  orange sensor crop  →  HT "
            f"{'aggregate output' if aggregate_mode else 'old/new comparison'}\n"
            f"map revision {self.global_count} | vehicle=({robot_x:.2f}, {robot_y:.2f}) m",
            fontsize=17,
        )
        path = self.frames_dir / f"frame_{self.saved_count:06d}.png"
        figure.savefig(path, dpi=100)
        plt.close(figure)

        finite_local = np.isfinite(local_height)
        aligned_cells, mask_agreement, max_alignment_error = (0, float("nan"), float("nan"))
        if stamp_ns(global_message) == key:
            aligned_cells, mask_agreement, max_alignment_error = compare_global_local(
                global_message,
                global_height,
                local_message,
                local_height,
            )
            if np.isfinite(mask_agreement) and np.isfinite(max_alignment_error):
                self.alignment_results.append((mask_agreement, max_alignment_error))
        self._manifest.writerow(
            {
                "gif_frame_index": self.saved_count,
                "source_ht_index": self.ht_count,
                "stamp_ns": key,
                "global_revision_frame": self.global_count,
                "local_center_x_m": robot_x,
                "local_center_y_m": robot_y,
                "local_finite_cells": int(finite_local.sum()),
                "ht_valid_cells": int(valid.sum()),
                "global_local_common_cells": aligned_cells,
                "global_local_mask_agreement": mask_agreement,
                "global_local_max_abs_error_m": max_alignment_error,
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
        gif_path = self.output_dir / "global_to_local_to_ht.gif"
        errors: list[str] = []
        if self.frame_paths:
            try:
                encode_gif(self.frame_paths, gif_path, self.gif_frame_ms)
            except Exception as exc:
                errors.append(f"GIF encoding failed: {exc}")
        else:
            errors.append("No synchronized HT/local frames were recorded")
        if not self.alignment_results:
            errors.append("No same-stamp global/local pair was available for alignment validation")
        else:
            minimum_agreement = min(value[0] for value in self.alignment_results)
            maximum_error = max(value[1] for value in self.alignment_results)
            if minimum_agreement < 1.0 or maximum_error > 1e-6:
                errors.append(
                    "Global/local alignment failed: "
                    f"min_mask_agreement={minimum_agreement}, max_error_m={maximum_error}"
                )

        summary = {
            "status": "passed" if not errors else "failed",
            "output_dir": str(self.output_dir),
            "elapsed_wall_seconds": time.time() - self.started,
            "received_global_frames": self.global_count,
            "received_local_frames": self.local_count,
            "received_ht_frames": self.ht_count,
            "saved_gif_frames": self.saved_count,
            "skipped_by_stride": self.skipped_stride,
            "skipped_missing_local": self.skipped_missing_local,
            "gif_frame_duration_ms": self.gif_frame_ms,
            "conservative_threshold": CONSERVATIVE_THRESHOLD,
            "gif_file": str(gif_path) if gif_path.exists() else "",
            "global_local_alignment_samples": len(self.alignment_results),
            "global_local_min_mask_agreement": (
                min(value[0] for value in self.alignment_results)
                if self.alignment_results
                else None
            ),
            "global_local_max_abs_error_m": (
                max(value[1] for value in self.alignment_results)
                if self.alignment_results
                else None
            ),
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
        gif_path = arguments.output_dir / "global_to_local_to_ht.gif"
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
    node = GlobalHtGifRecorder(
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
