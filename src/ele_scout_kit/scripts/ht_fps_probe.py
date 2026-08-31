#!/usr/bin/env python3
"""Record lightweight receive timestamps for the global-local and HT topics."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import statistics
import time

from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


LOCAL_TOPIC = "/elevation_mapping_node/elevation_map_global_local"
HT_TOPIC = "/ht_traversability/global_local"


def stamp_ns(message: GridMap) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def rate_summary(receive_times_ns: list[int]) -> dict[str, float | int | None]:
    if len(receive_times_ns) < 2:
        return {"messages": len(receive_times_ns), "mean_hz": None, "median_hz": None}
    intervals = [
        (right - left) / 1_000_000_000.0
        for left, right in zip(receive_times_ns, receive_times_ns[1:])
        if right > left
    ]
    return {
        "messages": len(receive_times_ns),
        "mean_hz": (len(receive_times_ns) - 1)
        / ((receive_times_ns[-1] - receive_times_ns[0]) / 1_000_000_000.0),
        "median_hz": 1.0 / statistics.median(intervals) if intervals else None,
    }


class HtFpsProbe(Node):
    def __init__(self, csv_path: Path) -> None:
        super().__init__("ht_fps_probe")
        self.csv_path = csv_path.resolve()
        self.events: list[tuple[str, int, int, int]] = []
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            GridMap, LOCAL_TOPIC, lambda message: self._record("local", message), qos
        )
        self.create_subscription(
            GridMap, HT_TOPIC, lambda message: self._record("ht", message), qos
        )
        self.get_logger().info(
            f"Measuring receive FPS: local={LOCAL_TOPIC}, ht={HT_TOPIC}"
        )

    def _record(self, stream: str, message: GridMap) -> None:
        self.events.append((stream, time.monotonic_ns(), time.time_ns(), stamp_ns(message)))

    def finalize(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                ["stream", "receive_monotonic_ns", "receive_wall_ns", "stamp_ns"]
            )
            writer.writerows(self.events)
        by_stream = {
            stream: [event[1] for event in self.events if event[0] == stream]
            for stream in ("local", "ht")
        }
        summary = {
            "csv_file": str(self.csv_path),
            "local": rate_summary(by_stream["local"]),
            "ht": rate_summary(by_stream["ht"]),
        }
        summary_path = self.csv_path.with_suffix(".summary.json")
        summary_path.write_text(
            json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        self.get_logger().info(json.dumps(summary, ensure_ascii=False))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_csv", type=Path)
    return parser.parse_args(rclpy.utilities.remove_ros_args()[1:])


def main() -> None:
    arguments = parse_args()
    rclpy.init()
    node = HtFpsProbe(arguments.output_csv)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
