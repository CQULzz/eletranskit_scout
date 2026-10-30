"""ROS 2 node bridging filtered elevation GridMaps to the HT student."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
import torch

from .core import (
    RECEPTIVE_FIELD,
    build_conservative_traversability_layer,
    build_direction_cost_layers,
    build_output_layers,
    embed_center,
    nearest_fill_nan,
    prepare_observation_height,
    add_unobserved_prior_layers,
    bounded_median_height,
    valid_window_ratio,
    validate_direction_order,
)
from .gridmap_codec import decode_layer, encode_layer
from .model import load_student_model


class HTTraversabilityNode(Node):
    def __init__(self) -> None:
        super().__init__("ht_traversability_node")

        self.declare_parameter("input_topic", "/elevation_mapping_node/elevation_map_filter")
        self.declare_parameter("input_layer", "inpaint")
        self.declare_parameter("unobserved_is_free", False)
        self.declare_parameter("height_median_kernel", 1)
        self.declare_parameter("height_median_max_delta", .04)
        self.declare_parameter("height_median_max_relief", .12)
        self.declare_parameter("output_topic", "/ht_traversability/traversability_8dir")
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cuda")
        self.declare_parameter("allow_cpu_fallback", False)
        self.declare_parameter("expected_resolution", 0.04)
        self.declare_parameter("resolution_tolerance", 0.001)
        self.declare_parameter("inference_fps", 5.0)
        self.declare_parameter("min_valid_ratio", 0.70)
        self.declare_parameter("direction_channel_order", [0, 1, 2, 3, 4, 5, 6, 7])
        self.declare_parameter("aggregate_only", False)
        self.declare_parameter("publish_direction_probabilities", True)
        self.declare_parameter("publish_direction_costmaps", False)
        self.declare_parameter("publish_merged_traversability", False)
        self.declare_parameter("publish_valid_layer", True)
        self.declare_parameter("publish_height_layer", True)
        self.declare_parameter("traversability_threshold", 0.5)
        self.declare_parameter("warmup_rows", 250)
        self.declare_parameter("warmup_cols", 250)
        self.declare_parameter("warmup_iterations", 2)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.input_layer = str(self.get_parameter("input_layer").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.expected_resolution = float(self.get_parameter("expected_resolution").value)
        self.resolution_tolerance = float(self.get_parameter("resolution_tolerance").value)
        self.inference_fps = float(self.get_parameter("inference_fps").value)
        self.min_valid_ratio = float(self.get_parameter("min_valid_ratio").value)
        self.aggregate_only = bool(self.get_parameter("aggregate_only").value)
        self.publish_direction_probabilities = bool(
            self.get_parameter("publish_direction_probabilities").value
        )
        self.publish_direction_costmaps = bool(
            self.get_parameter("publish_direction_costmaps").value
        )
        self.publish_merged_traversability = bool(
            self.get_parameter("publish_merged_traversability").value
        )
        self.publish_valid_layer = bool(self.get_parameter("publish_valid_layer").value)
        self.publish_height_layer = bool(self.get_parameter("publish_height_layer").value)
        self.traversability_threshold = float(
            self.get_parameter("traversability_threshold").value
        )
        self.direction_order = validate_direction_order(
            self.get_parameter("direction_channel_order").value
        )

        if not 0.0 <= self.min_valid_ratio <= 1.0:
            raise ValueError("min_valid_ratio must be between 0 and 1")
        if self.expected_resolution <= 0.0:
            raise ValueError("expected_resolution must be positive")
        if self.resolution_tolerance < 0.0:
            raise ValueError("resolution_tolerance must not be negative")
        self._validate_traversability_threshold(self.traversability_threshold)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.device = self._select_device()
        model_path = self._resolve_model_path()
        self.get_logger().info(f"Loading HT student weight: {model_path}")
        self.model = load_student_model(str(model_path), self.device)
        self._warmup_model()

        self.publisher = self.create_publisher(GridMap, self.output_topic, 10)
        self.subscription = self.create_subscription(
            GridMap, self.input_topic, self._map_callback, 10
        )

        self._next_inference_time = 0.0
        self._last_status_time = 0.0
        self._last_error_text: Optional[str] = None
        self._last_error_time = 0.0
        self.get_logger().info(
            "Ready: input=%s layer=%s output=%s device=%s fps=%.2f "
            "probabilities=%s costs=%s merged=%s valid=%s height=%s threshold=%.3f"
            % (
                self.input_topic,
                self.input_layer,
                self.output_topic,
                self.device,
                self.inference_fps,
                self.publish_direction_probabilities,
                self.publish_direction_costmaps,
                self.publish_merged_traversability,
                self.publish_valid_layer,
                self.publish_height_layer,
                self.traversability_threshold,
            )
        )

    @staticmethod
    def _validate_traversability_threshold(value: float) -> None:
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError("traversability_threshold must be between 0 and 1")

    def _on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        """Allow the aggregate switch and threshold to be changed at runtime."""

        aggregate_only = self.aggregate_only
        threshold = self.traversability_threshold
        for parameter in parameters:
            if parameter.name == "aggregate_only":
                if parameter.type_ != Parameter.Type.BOOL:
                    return SetParametersResult(
                        successful=False, reason="aggregate_only must be a boolean"
                    )
                aggregate_only = bool(parameter.value)
            elif parameter.name == "traversability_threshold":
                if parameter.type_ != Parameter.Type.DOUBLE:
                    return SetParametersResult(
                        successful=False,
                        reason="traversability_threshold must be a floating-point value",
                    )
                threshold = float(parameter.value)

        try:
            self._validate_traversability_threshold(threshold)
        except ValueError as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        changed = (
            aggregate_only != self.aggregate_only
            or threshold != self.traversability_threshold
        )
        self.aggregate_only = aggregate_only
        self.traversability_threshold = threshold
        if changed:
            self.get_logger().info(
                "HT output mode updated: mode=%s threshold=%.3f"
                % (
                    "aggregate_only" if aggregate_only else "eight_direction",
                    threshold,
                )
            )
        return SetParametersResult(successful=True)

    def _select_device(self) -> torch.device:
        requested = str(self.get_parameter("device").value).strip().lower()
        allow_fallback = bool(self.get_parameter("allow_cpu_fallback").value)
        if requested == "auto":
            requested = "cuda" if torch.cuda.is_available() else "cpu"
        if requested.startswith("cuda") and not torch.cuda.is_available():
            if not allow_fallback:
                raise RuntimeError(
                    "CUDA was requested but torch.cuda.is_available() is false. "
                    "Install the NVIDIA Jetson PyTorch build, or explicitly enable CPU fallback."
                )
            self.get_logger().warning("CUDA unavailable; falling back to CPU")
            requested = "cpu"
        return torch.device(requested)

    def _resolve_model_path(self) -> Path:
        configured = str(self.get_parameter("model_path").value).strip()
        if configured:
            result = Path(configured).expanduser()
        else:
            share = Path(get_package_share_directory("ht_traversability_ros2"))
            result = share / "models" / "student_best_epoch_weights.pth"
        if not result.is_file():
            raise FileNotFoundError(f"HT model weight does not exist: {result}")
        return result.resolve()

    def _warmup_model(self) -> None:
        rows = int(self.get_parameter("warmup_rows").value)
        cols = int(self.get_parameter("warmup_cols").value)
        iterations = int(self.get_parameter("warmup_iterations").value)
        if iterations <= 0:
            return
        if rows < RECEPTIVE_FIELD or cols < RECEPTIVE_FIELD:
            raise ValueError(
                f"Warmup grid {rows}x{cols} is smaller than the receptive field"
            )
        started = time.perf_counter()
        warmup = torch.zeros((1, 1, rows, cols), dtype=torch.float32, device=self.device)
        with torch.no_grad():
            for _ in range(iterations):
                self.model(warmup)
        if self.device.type == "cuda":
            torch.cuda.synchronize(self.device)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.get_logger().info(
            f"HT model warmup complete: grid={rows}x{cols}, "
            f"iterations={iterations}, elapsed={elapsed_ms:.1f} ms"
        )

    def _map_callback(self, message: GridMap) -> None:
        now = time.monotonic()
        if self.inference_fps > 0.0 and now < self._next_inference_time:
            return
        if self.inference_fps > 0.0:
            # Advance the cadence rather than restarting it at each arrival.
            # Otherwise slightly early 10 Hz frames get dropped into a 5 Hz pattern.
            self._next_inference_time = max(self._next_inference_time + 1.0 / self.inference_fps, now)

        started = time.perf_counter()
        try:
            self._validate_resolution(float(message.info.resolution))
            if self.input_layer not in message.layers:
                raise KeyError(
                    f"Input layer '{self.input_layer}' is absent; available={list(message.layers)}"
                )
            layer_index = list(message.layers).index(self.input_layer)
            height = decode_layer(self.input_layer, message.data[layer_index])
            if min(height.shape) < RECEPTIVE_FIELD:
                raise ValueError(
                    f"Elevation grid {height.shape} is smaller than the "
                    f"{RECEPTIVE_FIELD}x{RECEPTIVE_FIELD} receptive field"
                )

            observed = None
            if self.get_parameter("unobserved_is_free").value:
                if "observation_count" not in message.layers:
                    raise ValueError("Optimistic prior requires observation_count")
                count = decode_layer("observation_count", message.data[
                    list(message.layers).index("observation_count")])
                filled, finite_mask = prepare_observation_height(height, count)
                observed = finite_mask
            else:
                filled, finite_mask = nearest_fill_nan(height)
            filled = bounded_median_height(filled, finite_mask,
                int(self.get_parameter("height_median_kernel").value),
                float(self.get_parameter("height_median_max_delta").value),
                float(self.get_parameter("height_median_max_relief").value))
            support_ratio = valid_window_ratio(finite_mask, RECEPTIVE_FIELD)
            valid_output = support_ratio >= self.min_valid_ratio

            input_tensor = torch.from_numpy(filled).unsqueeze(0).unsqueeze(0)
            input_tensor = input_tensor.to(self.device, dtype=torch.float32)
            with torch.no_grad():
                logits = self.model(input_tensor)
                if logits.ndim != 5 or tuple(logits.shape[:3]) != (1, 8, 2):
                    raise RuntimeError(f"Unexpected HT output shape: {tuple(logits.shape)}")
                rows_out, cols_out = int(logits.shape[3]), int(logits.shape[4])
                if (rows_out, cols_out) != valid_output.shape:
                    raise RuntimeError(
                        "Model output and validity window have different shapes: "
                        f"{(rows_out, cols_out)} vs {valid_output.shape}"
                    )
                probabilities = torch.softmax(logits, dim=2)[0, :, 1]
                probabilities = probabilities[list(self.direction_order)]
                probabilities_np = probabilities.cpu().numpy().astype(np.float32, copy=False)

            # Keep aggregate_only as a backward-compatible legacy switch. New
            # deployments should use the explicit publish_* parameters below.
            publish_probabilities = (
                self.publish_direction_probabilities and not self.aggregate_only
            )
            publish_merged = self.publish_merged_traversability or self.aggregate_only
            layers: dict[str, np.ndarray] = {}
            if publish_probabilities:
                layers.update(build_output_layers(probabilities_np, valid_output, height.shape))
            if self.publish_direction_costmaps:
                layers.update(
                    build_direction_cost_layers(
                        probabilities_np,
                        valid_output,
                        height.shape,
                        self.traversability_threshold,
                    )
                )
            if publish_merged:
                layers["ht_traversable"] = build_conservative_traversability_layer(
                    probabilities_np,
                    valid_output,
                    height.shape,
                    self.traversability_threshold,
                )
            if self.publish_valid_layer and "ht_valid" not in layers:
                layers["ht_valid"] = embed_center(
                    valid_output.astype(np.float32), height.shape, fill=0.0
                )
            if self.publish_height_layer:
                layers["ht_height"] = height.astype(np.float32, copy=False)
            if observed is not None:
                add_unobserved_prior_layers(layers, observed)
            if not layers:
                raise RuntimeError("HT output has no enabled layers")
            self.publisher.publish(self._make_output_message(message, layers))
            self._last_error_text = None

            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if now - self._last_status_time >= 5.0:
                valid_percent = 100.0 * float(valid_output.mean())
                self.get_logger().info(
                    f"HT inference {elapsed_ms:.1f} ms, grid={height.shape}, "
                    f"valid_output={valid_percent:.1f}% "
                    f"layers={list(layers)}"
                )
                self._last_status_time = now
        except Exception as exc:  # Keep the mapping pipeline alive on malformed frames.
            self._log_callback_error(str(exc), now)

    def _validate_resolution(self, actual: float) -> None:
        if not np.isfinite(actual):
            raise ValueError("GridMap resolution is not finite")
        if abs(actual - self.expected_resolution) > self.resolution_tolerance:
            raise ValueError(
                f"GridMap resolution {actual:.6f} m does not match model resolution "
                f"{self.expected_resolution:.6f} m"
            )

    @staticmethod
    def _make_output_message(
        source: GridMap, layers: dict[str, np.ndarray]
    ) -> GridMap:
        output = GridMap()
        output.header = source.header
        output.info = source.info
        output.layers = list(layers.keys())
        output.basic_layers = []
        output.data = [encode_layer(value) for value in layers.values()]
        output.outer_start_index = source.outer_start_index
        output.inner_start_index = source.inner_start_index
        return output

    def _log_callback_error(self, text: str, now: float) -> None:
        if text != self._last_error_text or now - self._last_error_time >= 5.0:
            self.get_logger().error(f"HT frame skipped: {text}")
            self._last_error_text = text
            self._last_error_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[HTTraversabilityNode] = None
    try:
        node = HTTraversabilityNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
