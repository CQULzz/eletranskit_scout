#!/usr/bin/env python3
"""Save synchronized local elevation and eight-direction HT diagnostic frames."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
import numpy as np
from PIL import Image
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from ht_traversability_ros2.core import nearest_fill_nan
from ht_traversability_ros2.gridmap_codec import decode_layer


LOCAL_TOPIC = "/elevation_mapping_node/elevation_map_global_local"
HT_TOPIC = "/ht_traversability/global_local"
ODOM_TOPIC = "/fixposition/odometry_enu"
TRAVERSABILITY_THRESHOLD = 0.15


def stamp_ns(message: GridMap) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def layers(message: GridMap) -> dict[str, np.ndarray]:
    result: dict[str, np.ndarray] = {}
    for name, data in zip(message.layers, message.data):
        matrix = decode_layer(name, data)
        # grid_map stores row=-X,column=-Y. Convert to image row=+Y,column=+X.
        result[name] = np.ascontiguousarray(np.flip(matrix, axis=(0, 1)).T)
    return result


def masked_cmap(name: str):
    result = plt.get_cmap(name).copy()
    result.set_bad("black")
    return result


def finite_stats(values: np.ndarray) -> tuple[int, float, float, float, float, float, float]:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return (0, *(float("nan") for _ in range(6)))
    return (
        int(finite.size),
        float(np.min(finite)),
        float(np.percentile(finite, 10)),
        float(np.median(finite)),
        float(np.mean(finite)),
        float(np.percentile(finite, 90)),
        float(np.max(finite)),
    )


def gradient_magnitude(height: np.ndarray, resolution: float) -> np.ndarray:
    finite = np.isfinite(height)
    if not finite.any():
        return np.full(height.shape, np.nan, dtype=np.float32)
    filled, _ = nearest_fill_nan(height)
    dy, dx = np.gradient(filled, resolution, resolution)
    gradient = np.hypot(dx, dy).astype(np.float32)
    gradient[~finite] = np.nan
    return gradient


def encode_gif(frame_paths: list[Path], output: Path, duration_ms: int) -> None:
    resampling = getattr(Image, "Resampling", Image)
    quantize = getattr(Image, "Quantize", Image)
    frames: list[Image.Image] = []
    for path in frame_paths:
        with Image.open(path) as source:
            image = source.convert("RGB")
            image.thumbnail((1600, 900), resampling.LANCZOS)
            frames.append(image.quantize(colors=192, method=quantize.MEDIANCUT))
    if frames:
        frames[0].save(
            output,
            save_all=True,
            append_images=frames[1:],
            duration=duration_ms,
            loop=0,
            optimize=False,
            disposal=2,
        )


class HtDiagnosticFrameRecorder(Node):
    def __init__(self, output_dir: Path, frame_stride: int, gif_frame_ms: int) -> None:
        super().__init__("ht_diagnostic_frame_recorder")
        self.output_dir = output_dir.resolve()
        self.frames_dir = self.output_dir / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=False)
        self.frame_stride = int(frame_stride)
        self.gif_frame_ms = int(gif_frame_ms)
        self.started = time.time()
        self.first_stamp: int | None = None
        self.local_count = 0
        self.ht_count = 0
        self.saved_count = 0
        self.skipped_stride = 0
        self.skipped_missing_local = 0
        self.local_by_stamp: dict[int, GridMap] = {}
        self.pending_ht: dict[int, tuple[GridMap, int]] = {}
        self.frame_paths: list[Path] = []
        self.times: list[float] = []
        self.z_p10: list[float] = []
        self.z_median: list[float] = []
        self.z_p90: list[float] = []
        self.dz_dt: list[float] = []
        self.heading_yaw: list[float] = []
        self._finalized = False
        self.latest_odometry: Odometry | None = None

        # The bag contains the complete FP_ECEF -> FP_ENU0 -> map -> odom and
        # FP_ECEF -> FP_POI -> base_footprint TF chains.  Looking up the
        # composed transform here makes the plotted arrow reflect recorded TF,
        # rather than assuming that the local crop's +x axis is vehicle-forward.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=180.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._manifest_handle = (self.output_dir / "manifest.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self._manifest = csv.DictWriter(
            self._manifest_handle,
            fieldnames=[
                "frame_index", "source_ht_index", "stamp_ns", "time_s",
                "local_center_x_m", "local_center_y_m", "finite_z_cells",
                "z_min_m", "z_p10_m", "z_median_m", "z_mean_m", "z_p90_m",
                "z_max_m", "z_gradient_median", "z_gradient_mean",
                "z_gradient_p95", "z_derivative_mps", "vehicle_yaw_rad",
                "vehicle_yaw_deg", "heading_tf", "ht_valid_cells",
                *(f"ht_dir_{index}_median" for index in range(8)), "png_file",
            ],
        )
        self._manifest.writeheader()

        local_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            # Plotting a twelve-panel PNG is intentionally much slower than
            # mapping. Never let this diagnostic subscriber apply DDS
            # backpressure to the real pipeline.
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        ht_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.create_subscription(GridMap, LOCAL_TOPIC, self._local, local_qos),
            self.create_subscription(GridMap, HT_TOPIC, self._ht, ht_qos),
            self.create_subscription(Odometry, ODOM_TOPIC, self._odometry, 50),
        ]
        self.get_logger().info(
            f"Recording detailed HT diagnostic frames under {self.output_dir}"
        )

    def _odometry(self, message: Odometry) -> None:
        self.latest_odometry = message

    def _local(self, message: GridMap) -> None:
        self.local_count += 1
        key = stamp_ns(message)
        pending = self.pending_ht.pop(key, None)
        if pending is not None:
            ht_message, source_index = pending
            self._save_safely(message, ht_message, key, source_index)
            return
        self.local_by_stamp[key] = message
        if len(self.local_by_stamp) > 300:
            for stale in sorted(self.local_by_stamp)[:-200]:
                del self.local_by_stamp[stale]

    def _ht(self, message: GridMap) -> None:
        self.ht_count += 1
        key = stamp_ns(message)
        if self.ht_count % self.frame_stride != 0:
            self.skipped_stride += 1
            return
        local = self.local_by_stamp.pop(key, None)
        if local is None:
            self.pending_ht[key] = (message, self.ht_count)
            if len(self.pending_ht) > 300:
                stale = sorted(self.pending_ht)[:-200]
                self.skipped_missing_local += len(stale)
                for stale_key in stale:
                    del self.pending_ht[stale_key]
            return
        self._save_safely(local, message, key, self.ht_count)

    def _save_safely(
        self, local: GridMap, ht: GridMap, key: int, source_index: int
    ) -> None:
        try:
            self._save_frame(local, ht, key, source_index)
        except Exception as exc:
            self.get_logger().error(f"Failed to render stamp {key}: {exc}")

    def _save_frame(
        self, local_message: GridMap, ht_message: GridMap, key: int, source_index: int
    ) -> None:
        local_layers = layers(local_message)
        ht_layers = layers(ht_message)
        height = local_layers["elevation"]
        probability = np.stack([ht_layers[f"ht_dir_{index}"] for index in range(8)])
        valid = ht_layers["ht_valid"] > 0.5
        probability[:, ~valid] = np.nan
        binary_cost = np.where(
            valid[np.newaxis, :, :],
            (probability < TRAVERSABILITY_THRESHOLD).astype(np.float32),
            np.nan,
        )
        resolution = float(local_message.info.resolution)
        gradient = gradient_magnitude(height, resolution)
        z_stats = finite_stats(height)
        gradient_stats = finite_stats(gradient)
        if self.first_stamp is None:
            self.first_stamp = key
        relative_time = (key - self.first_stamp) / 1_000_000_000.0
        self.times.append(relative_time)
        self.z_p10.append(z_stats[2])
        self.z_median.append(z_stats[3])
        self.z_p90.append(z_stats[5])
        finite_gradient = gradient[np.isfinite(gradient)]
        gradient_p95 = (
            float(np.percentile(finite_gradient, 95)) if finite_gradient.size else float("nan")
        )
        if len(self.times) < 2 or not np.isfinite(self.z_median[-1]):
            dz_dt = float("nan")
        else:
            dt = self.times[-1] - self.times[-2]
            previous = self.z_median[-2]
            dz_dt = (
                (self.z_median[-1] - previous) / dt
                if dt > 0.0 and np.isfinite(previous) else float("nan")
            )
        self.dz_dt.append(dz_dt)

        heading_tf = "odom<-base_footprint"
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time(), timeout=Duration(seconds=0.0)
            )
            q = transform.transform.rotation
            yaw = float(np.arctan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            ))
        except TransformException as exc:
            # Stable replay publishes this odometry from the same recorded
            # Fixposition pose as FP_ECEF -> FP_POI.  Its ENU quaternion is the
            # composed TF orientation and remains available even when the
            # recorder's TF callbacks trail the expensive plotting callback.
            if self.latest_odometry is not None:
                q = self.latest_odometry.pose.pose.orientation
                yaw = float(np.arctan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                ))
                heading_tf = "FP_ENU0<-FP_POI (recorded odometry fallback)"
            else:
                yaw = float("nan")
                heading_tf = f"unavailable: {exc}"
                self.get_logger().warning(
                    f"TF heading unavailable for stamp {key}: {exc}",
                    throttle_duration_sec=2.0,
                )
        self.heading_yaw.append(yaw)

        half_x = float(local_message.info.length_x) / 2.0
        half_y = float(local_message.info.length_y) / 2.0
        plot_extent = (-half_x, half_x, -half_y, half_y)
        probability_cmap = ListedColormap(["#1a9641", "#d7191c"])
        probability_cmap.set_bad("black")
        probability_norm = BoundaryNorm([-0.5, 0.5, 1.5], probability_cmap.N)
        height_cmap = masked_cmap("terrain")
        gradient_cmap = masked_cmap("magma")
        figure = plt.figure(figsize=(18, 10.7), constrained_layout=True)
        grid = figure.add_gridspec(3, 4, height_ratios=(1.0, 1.0, 1.12))
        direction_axes = [figure.add_subplot(grid[row, column]) for row in range(2) for column in range(4)]
        for index, axis in enumerate(direction_axes):
            image = axis.imshow(
                np.ma.masked_invalid(binary_cost[index]), origin="lower",
                extent=plot_extent, cmap=probability_cmap, norm=probability_norm,
            )
            axis.set_title(f"HT cost dir {index} binary (threshold {TRAVERSABILITY_THRESHOLD:.2f})")
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.set_aspect("equal")
            if np.isfinite(yaw):
                arrow_length = 0.22 * min(2.0 * half_x, 2.0 * half_y)
                axis.arrow(
                    0.0, 0.0, arrow_length * np.cos(yaw), arrow_length * np.sin(yaw),
                    width=0.025, head_width=0.22, head_length=0.28,
                    facecolor="deepskyblue", edgecolor="black", length_includes_head=True,
                    zorder=5,
                )
                axis.text(0.0, 0.0, " vehicle", color="white", fontsize=7,
                          ha="left", va="bottom", zorder=6)
        figure.colorbar(
            image, ax=direction_axes, fraction=0.012, pad=0.012,
            ticks=[0, 1], label="0 = free, 1 = blocked",
        )

        elevation_axis = figure.add_subplot(grid[2, 0])
        elevation_image = elevation_axis.imshow(
            np.ma.masked_invalid(height), origin="lower", extent=plot_extent,
            cmap=height_cmap, vmin=4.5, vmax=7.8,
        )
        elevation_axis.plot(0.0, 0.0, marker="x", color="magenta", markersize=9, markeredgewidth=2)
        elevation_axis.set_title("Local z elevation (fixed 4.5-7.8 m)")
        figure.colorbar(elevation_image, ax=elevation_axis, fraction=0.046, pad=0.03, label="z [m]")

        gradient_axis = figure.add_subplot(grid[2, 1])
        gradient_image = gradient_axis.imshow(
            np.ma.masked_invalid(gradient), origin="lower", extent=plot_extent,
            cmap=gradient_cmap, vmin=0.0, vmax=3.0,
        )
        gradient_axis.plot(0.0, 0.0, marker="x", color="cyan", markersize=9, markeredgewidth=2)
        gradient_axis.set_title("Local |grad z| (fixed 0-3 m/m)")
        figure.colorbar(gradient_image, ax=gradient_axis, fraction=0.046, pad=0.03, label="m/m")
        for axis in (elevation_axis, gradient_axis):
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.set_aspect("equal")

        z_axis = figure.add_subplot(grid[2, 2])
        z_axis.plot(self.times, self.z_median, color="tab:blue", label="median")
        z_axis.fill_between(self.times, self.z_p10, self.z_p90, color="tab:blue", alpha=0.18, label="p10-p90")
        z_axis.scatter([relative_time], [self.z_median[-1]], color="tab:red", s=30, zorder=3)
        z_axis.set_title("z over time")
        z_axis.set_xlabel("time [s]")
        z_axis.set_ylabel("z [m]")
        z_axis.grid(True, alpha=0.3)
        z_axis.legend(loc="upper right", fontsize=8)

        gradient_time_axis = figure.add_subplot(grid[2, 3])
        gradient_time_axis.plot(self.times, self.dz_dt, color="tab:purple", label="d(median z)/dt")
        gradient_time_axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        gradient_time_axis.scatter([relative_time], [self.dz_dt[-1]], color="tab:red", s=30, zorder=3)
        gradient_time_axis.set_title("z derivative over time")
        gradient_time_axis.set_xlabel("time [s]")
        gradient_time_axis.set_ylabel("dz/dt [m/s]")
        gradient_time_axis.grid(True, alpha=0.3)
        gradient_time_axis.legend(loc="upper right", fontsize=8)

        center_x = float(local_message.info.pose.position.x)
        center_y = float(local_message.info.pose.position.y)
        figure.suptitle(
            "HT eight-direction BINARY costmap | "
            f"frame {self.saved_count:06d} | t={relative_time:.2f}s | "
            f"center=({center_x:.2f}, {center_y:.2f}) m | "
            f"TF yaw={np.degrees(yaw):.1f} deg",
            fontsize=16,
        )
        path = self.frames_dir / f"frame_{self.saved_count:06d}.png"
        figure.savefig(path, dpi=100)
        plt.close(figure)

        direction_medians = []
        for index in range(8):
            finite = probability[index][np.isfinite(probability[index])]
            direction_medians.append(float(np.median(finite)) if finite.size else float("nan"))
        row = {
            "frame_index": self.saved_count,
            "source_ht_index": source_index,
            "stamp_ns": key,
            "time_s": relative_time,
            "local_center_x_m": center_x,
            "local_center_y_m": center_y,
            "finite_z_cells": z_stats[0],
            "z_min_m": z_stats[1], "z_p10_m": z_stats[2],
            "z_median_m": z_stats[3], "z_mean_m": z_stats[4],
            "z_p90_m": z_stats[5], "z_max_m": z_stats[6],
            "z_gradient_median": gradient_stats[3],
            "z_gradient_mean": gradient_stats[4],
            "z_gradient_p95": gradient_p95,
            "z_derivative_mps": dz_dt,
            "vehicle_yaw_rad": yaw,
            "vehicle_yaw_deg": float(np.degrees(yaw)),
            "heading_tf": heading_tf,
            "ht_valid_cells": int(valid.sum()),
            "png_file": str(path.relative_to(self.output_dir)),
        }
        row.update({f"ht_dir_{index}_median": direction_medians[index] for index in range(8)})
        self._manifest.writerow(row)
        self._manifest_handle.flush()
        self.frame_paths.append(path)
        self.saved_count += 1
        if self.saved_count % 10 == 0:
            self.get_logger().info(f"Saved detailed PNG frames: {self.saved_count}")

    def finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        self.skipped_missing_local += len(self.pending_ht)
        self.pending_ht.clear()
        self._manifest_handle.flush()
        self._manifest_handle.close()
        gif_path = self.output_dir / "ht_diagnostic.gif"
        errors: list[str] = []
        try:
            encode_gif(self.frame_paths, gif_path, self.gif_frame_ms)
        except Exception as exc:
            errors.append(f"GIF encoding failed: {exc}")
        if not self.frame_paths:
            errors.append("No synchronized local/HT frames were recorded")
        summary = {
            "status": "passed" if not errors else "failed",
            "output_dir": str(self.output_dir),
            "elapsed_wall_seconds": time.time() - self.started,
            "received_local_frames": self.local_count,
            "received_ht_frames": self.ht_count,
            "saved_png_frames": self.saved_count,
            "skipped_by_stride": self.skipped_stride,
            "skipped_missing_local": self.skipped_missing_local,
            "gif_frame_duration_ms": self.gif_frame_ms,
            "gif_file": str(gif_path) if gif_path.exists() else "",
            "first_frame": str(self.frame_paths[0]) if self.frame_paths else "",
            "last_frame": str(self.frame_paths[-1]) if self.frame_paths else "",
            "errors": errors,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        self.get_logger().info(json.dumps(summary, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--gif-frame-ms", type=int, default=120)
    arguments, ros_arguments = parser.parse_known_args()
    if arguments.frame_stride <= 0 or arguments.gif_frame_ms <= 0:
        parser.error("frame stride and GIF duration must be positive")
    rclpy.init(args=ros_arguments)
    node = HtDiagnosticFrameRecorder(
        arguments.output_dir, arguments.frame_stride, arguments.gif_frame_ms
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
