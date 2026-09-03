#!/usr/bin/env python3
"""Play a ROS 2 sqlite3 bag one logical frame per key press.

The original bag is never modified.  The script builds a small metadata index,
starts the official rosbag2 player paused, republishes a complete static TF set,
and advances the player through its service API.
"""

from __future__ import annotations

import argparse
import bisect
import dataclasses
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import sqlite3
import struct
import subprocess
import sys
import termios
import time
import tty
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSPresetProfiles, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosbag2_interfaces.srv import Burst
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import UInt64
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


CLOUD_TOPIC = "/rslidar_points"
TF_TOPIC = "/tf"
STATIC_TF_TOPIC = "/tf_static"
ODOM_TOPIC = "/fixposition/odometry_enu"
RAW_MAP_TOPIC = "/elevation_mapping_node/elevation_map_raw"
FILTER_MAP_TOPIC = "/elevation_mapping_node/elevation_map_filter"

MODE_TOPICS = {
    # Stable mode uses the official C++ player for the large cloud only.  This
    # controller publishes timestamp-bracketing TF before asking for one cloud.
    "stable": (CLOUD_TOPIC,),
    # Faithful mode preserves the recorder arrival order for the relevant live
    # inputs and advances exactly through one cloud per burst.
    "faithful": (TF_TOPIC, ODOM_TOPIC, CLOUD_TOPIC),
    # Recorded mode is for inspecting the maps already stored in the bag.  Map
    # topics are remapped so they cannot collide with a running mapper.
    "recorded": (
        TF_TOPIC,
        ODOM_TOPIC,
        CLOUD_TOPIC,
        RAW_MAP_TOPIC,
        FILTER_MAP_TOPIC,
    ),
}

MODE_ANCHOR = {
    "stable": CLOUD_TOPIC,
    "faithful": CLOUD_TOPIC,
    "recorded": FILTER_MAP_TOPIC,
}

RECORD_REMAPS = {
    RAW_MAP_TOPIC: "/recorded/elevation_map_raw",
    FILTER_MAP_TOPIC: "/recorded/elevation_map_filter",
}

INDEX_VERSION = 2


@dataclasses.dataclass(frozen=True)
class Frame:
    number: int
    record_ns: int
    header_ns: int
    burst_count: int
    anchor_topic: str


@dataclasses.dataclass(frozen=True)
class BagLayout:
    bag_dir: Path
    databases: Tuple[Path, ...]


def _metadata_database_names(metadata_path: Path) -> List[str]:
    """Read relative database paths without requiring PyYAML."""
    names: List[str] = []
    in_paths = False
    for raw_line in metadata_path.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if stripped.startswith("relative_file_paths:"):
            in_paths = True
            continue
        if in_paths and stripped.startswith("-"):
            names.append(stripped[1:].strip().strip("'\""))
            continue
        if in_paths and stripped and not raw_line.startswith((" ", "\t")):
            break
    return names


def resolve_bag(path_text: str) -> BagLayout:
    path = Path(path_text).expanduser().resolve()
    if path.is_file() and path.suffix == ".db3":
        return BagLayout(path.parent, (path,))
    if not path.is_dir():
        raise FileNotFoundError(f"Bag path does not exist: {path}")

    metadata = path / "metadata.yaml"
    database_paths: List[Path] = []
    if metadata.exists():
        database_paths = [path / name for name in _metadata_database_names(metadata)]
        database_paths = [candidate for candidate in database_paths if candidate.exists()]
    if not database_paths:
        database_paths = sorted(path.glob("*.db3"))
    if not database_paths:
        raise FileNotFoundError(f"No .db3 database found in {path}")
    return BagLayout(path, tuple(database_paths))


def _readonly_connection(database: Path) -> sqlite3.Connection:
    return sqlite3.connect(f"file:{database}?mode=ro", uri=True)


def _bag_signature(layout: BagLayout) -> str:
    payload = []
    for database in layout.databases:
        stat = database.stat()
        payload.append((database.name, stat.st_size, stat.st_mtime_ns))
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()[:16]


def _decode_header_stamp(prefix: bytes) -> int:
    """Decode the first std_msgs/Header stamp in a CDR serialized message."""
    if len(prefix) < 12:
        raise ValueError("Serialized message is too short to contain a Header")
    # CDR representation id 0x0001/0x0003 is little endian.  ROS 2 Humble's
    # sqlite3 bags on this machine use 00 01 00 00.
    endian = "<" if prefix[1] & 0x01 else ">"
    sec, nanosec = struct.unpack_from(f"{endian}iI", prefix, 4)
    return int(sec) * 1_000_000_000 + int(nanosec)


def _topic_rows(
    layout: BagLayout,
    selected_topics: Sequence[str],
    anchor_topic: str,
) -> List[Tuple[int, int, str, Optional[bytes]]]:
    rows: List[Tuple[int, int, str, Optional[bytes]]] = []
    placeholders = ",".join("?" for _ in selected_topics)
    query = f"""
        SELECT m.timestamp, m.id, t.name,
               CASE WHEN t.name = ? THEN substr(m.data, 1, 12) ELSE NULL END
        FROM messages AS m
        JOIN topics AS t ON t.id = m.topic_id
        WHERE t.name IN ({placeholders})
        ORDER BY m.timestamp, m.id
    """
    for database_index, database in enumerate(layout.databases):
        with _readonly_connection(database) as connection:
            for timestamp, message_id, topic, prefix in connection.execute(
                query, (anchor_topic, *selected_topics)
            ):
                # Include database order in the id so split bags remain stable.
                global_id = database_index * 10**15 + int(message_id)
                rows.append((int(timestamp), global_id, str(topic), prefix))
    rows.sort(key=lambda row: (row[0], row[1]))
    return rows


def build_frames(layout: BagLayout, mode: str) -> List[Frame]:
    topics = MODE_TOPICS[mode]
    anchor = MODE_ANCHOR[mode]
    rows = _topic_rows(layout, topics, anchor)
    if not rows:
        raise RuntimeError(f"No selected messages found in {layout.bag_dir}")

    available_topics = {row[2] for row in rows}
    missing = set(topics) - available_topics
    if missing:
        raise RuntimeError(f"Bag is missing required topic(s): {sorted(missing)}")

    if mode == "stable":
        frames = []
        for number, (record_ns, _message_id, topic, prefix) in enumerate(rows, start=1):
            if prefix is None:
                raise RuntimeError(f"Missing serialized header for {topic}")
            frames.append(
                Frame(number, record_ns, _decode_header_stamp(prefix), 1, anchor)
            )
        return frames

    frames: List[Frame] = []
    pending_count = 0
    previous_anchor_header_ns: Optional[int] = None
    cursor = 0
    while cursor < len(rows):
        timestamp = rows[cursor][0]
        bucket = []
        while cursor < len(rows) and rows[cursor][0] == timestamp:
            bucket.append(rows[cursor])
            cursor += 1
        pending_count += len(bucket)
        anchors = [row for row in bucket if row[2] == anchor]
        if len(anchors) > 1:
            raise RuntimeError(
                f"Multiple {anchor} messages share storage timestamp {timestamp}; "
                "cannot guarantee one-anchor bursts."
            )
        if anchors:
            prefix = anchors[0][3]
            if prefix is None:
                raise RuntimeError(f"Missing serialized header for {anchor}")
            header_ns = _decode_header_stamp(prefix)
            # Recorded filter maps were published repeatedly with an unchanged
            # source header.  Treat those as one visual state: a key advances
            # through the repeats and stops at the next genuinely new map.
            if mode == "recorded" and header_ns == previous_anchor_header_ns:
                continue
            frames.append(
                Frame(
                    len(frames) + 1,
                    int(timestamp),
                    header_ns,
                    pending_count,
                    anchor,
                )
            )
            pending_count = 0
            previous_anchor_header_ns = header_ns
    if not frames:
        raise RuntimeError(f"No anchor messages found on {anchor}")
    return frames


def index_path(index_dir: Path, layout: BagLayout, mode: str) -> Path:
    safe_name = layout.bag_dir.name.replace(os.sep, "_")
    return index_dir / f"{safe_name}.{mode}.json"


def load_or_build_frames(
    layout: BagLayout,
    mode: str,
    index_dir: Path,
    force: bool = False,
) -> List[Frame]:
    index_dir.mkdir(parents=True, exist_ok=True)
    target = index_path(index_dir, layout, mode)
    signature = _bag_signature(layout)
    if target.exists() and not force:
        try:
            document = json.loads(target.read_text(encoding="utf-8"))
            if (
                document.get("index_version") == INDEX_VERSION
                and document.get("signature") == signature
                and document.get("mode") == mode
            ):
                return [Frame(**item) for item in document["frames"]]
        except (ValueError, KeyError, TypeError):
            pass

    frames = build_frames(layout, mode)
    document = {
        "index_version": INDEX_VERSION,
        "bag": str(layout.bag_dir),
        "signature": signature,
        "mode": mode,
        "anchor_topic": MODE_ANCHOR[mode],
        "selected_topics": list(MODE_TOPICS[mode]),
        "frames": [dataclasses.asdict(frame) for frame in frames],
    }
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(json.dumps(document, indent=2), encoding="utf-8")
    temporary.replace(target)
    return frames


def _serialized_rows(layout: BagLayout, topic: str) -> Iterable[Tuple[int, bytes]]:
    collected: List[Tuple[int, int, bytes]] = []
    for database_index, database in enumerate(layout.databases):
        with _readonly_connection(database) as connection:
            query = """
                SELECT m.timestamp, m.id, m.data
                FROM messages AS m
                JOIN topics AS t ON t.id = m.topic_id
                WHERE t.name = ?
                ORDER BY m.timestamp, m.id
            """
            for timestamp, message_id, data in connection.execute(query, (topic,)):
                global_id = database_index * 10**15 + int(message_id)
                collected.append((int(timestamp), global_id, bytes(data)))
    collected.sort(key=lambda row: (row[0], row[1]))
    for timestamp, _message_id, data in collected:
        yield timestamp, data


def _transform_signature(transform: TransformStamped) -> Tuple[float, ...]:
    tr = transform.transform.translation
    rot = transform.transform.rotation
    return (tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w)


def load_static_transforms(layout: BagLayout) -> List[TransformStamped]:
    unique: Dict[Tuple[str, str], TransformStamped] = {}
    signatures: Dict[Tuple[str, str], Tuple[float, ...]] = {}
    for _record_ns, data in _serialized_rows(layout, STATIC_TF_TOPIC):
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            signature = _transform_signature(transform)
            if edge in signatures:
                if any(
                    not math.isclose(a, b, rel_tol=0.0, abs_tol=1e-9)
                    for a, b in zip(signatures[edge], signature)
                ):
                    raise RuntimeError(f"Conflicting static transforms recorded for {edge}")
                continue
            unique[edge] = transform
            signatures[edge] = signature
    if not unique:
        raise RuntimeError("Bag has no /tf_static transforms")
    return [unique[key] for key in sorted(unique)]


class StableInputs:
    """Small dynamic inputs published ahead of each cloud in stable mode."""

    def __init__(self, layout: BagLayout) -> None:
        self.tf_by_edge: Dict[
            Tuple[str, str], Tuple[List[int], List[TransformStamped]]
        ] = {}
        working: Dict[Tuple[str, str], List[Tuple[int, TransformStamped]]] = {}
        for _record_ns, data in _serialized_rows(layout, TF_TOPIC):
            message = deserialize_message(data, TFMessage)
            for transform in message.transforms:
                stamp = transform.header.stamp
                stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                edge = (transform.header.frame_id, transform.child_frame_id)
                working.setdefault(edge, []).append((stamp_ns, transform))
        for edge, entries in working.items():
            entries.sort(key=lambda entry: entry[0])
            self.tf_by_edge[edge] = (
                [entry[0] for entry in entries],
                [entry[1] for entry in entries],
            )

        odometry: List[Tuple[int, Odometry]] = []
        for _record_ns, data in _serialized_rows(layout, ODOM_TOPIC):
            message = deserialize_message(data, Odometry)
            stamp = message.header.stamp
            stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            odometry.append((stamp_ns, message))
        odometry.sort(key=lambda entry: entry[0])
        self.odom_times = [entry[0] for entry in odometry]
        self.odom_messages = [entry[1] for entry in odometry]

        preferred_edge = ("FP_ECEF", "FP_POI")
        self.critical_edge = preferred_edge if preferred_edge in self.tf_by_edge else next(
            iter(self.tf_by_edge), None
        )

    @staticmethod
    def _indices_around(times: Sequence[int], target_ns: int) -> Tuple[Optional[int], Optional[int]]:
        right = bisect.bisect_left(times, target_ns)
        before = right - 1 if right > 0 else None
        if right < len(times) and times[right] == target_ns:
            return right, right
        after = right if right < len(times) else None
        return before, after

    def messages_for(
        self, target_ns: int
    ) -> Tuple[List[TransformStamped], Optional[Odometry], bool]:
        selected: List[TransformStamped] = []
        critical_bracketed = False
        for edge, (times, transforms) in self.tf_by_edge.items():
            before, after = self._indices_around(times, target_ns)
            indices = []
            if before is not None:
                indices.append(before)
            if after is not None and after != before:
                indices.append(after)
            selected.extend(transforms[index] for index in indices)
            if edge == self.critical_edge:
                critical_bracketed = before is not None and after is not None

        odometry_message = None
        if self.odom_times:
            index = bisect.bisect_right(self.odom_times, target_ns) - 1
            if index < 0:
                index = 0
            odometry_message = self.odom_messages[index]
        return selected, odometry_message, critical_bracketed


class StepController(Node):
    def __init__(
        self,
        stable_inputs: Optional[StableInputs],
        acknowledgement_topic: str,
        receipt_topic: str,
        direct_cloud_publish: bool,
    ) -> None:
        super().__init__("rosbag_frame_controller")
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.burst_client = (
            None
            if direct_cloud_publish
            else self.create_client(Burst, "/rosbag2_player/burst")
        )
        self.stable_inputs = stable_inputs
        self.acknowledgement_topic = acknowledgement_topic
        self.acknowledgement_count: Optional[int] = None
        self.acknowledgement_subscription = None
        if acknowledgement_topic:
            self.acknowledgement_subscription = self.create_subscription(
                UInt64,
                acknowledgement_topic,
                self._acknowledgement_callback,
                10,
            )
        self.receipt_topic = receipt_topic
        self.receipt_count: Optional[int] = None
        self.receipt_subscription = None
        if receipt_topic:
            self.receipt_subscription = self.create_subscription(
                UInt64,
                receipt_topic,
                self._receipt_callback,
                10,
            )
        self.cloud_publisher = None
        if direct_cloud_publish:
            self.cloud_publisher = self.create_publisher(
                PointCloud2,
                CLOUD_TOPIC,
                # Direct frame-controlled replay must not silently lose a
                # multi-megabyte cloud.  Reliable delivery also matches the
                # live mapping subscriber used by the kit.
                QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE),
            )

    def _acknowledgement_callback(self, message: UInt64) -> None:
        value = int(message.data)
        if self.acknowledgement_count is None or value > self.acknowledgement_count:
            self.acknowledgement_count = value

    def _receipt_callback(self, message: UInt64) -> None:
        value = int(message.data)
        if self.receipt_count is None or value > self.receipt_count:
            self.receipt_count = value

    def wait_for_acknowledgement_publisher(
        self, process: Optional[subprocess.Popen], timeout_sec: float = 20.0
    ) -> None:
        if self.acknowledgement_subscription is None:
            return
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if process is not None and process.poll() is not None:
                raise RuntimeError(f"rosbag2 player exited with code {process.returncode}")
            acknowledgement_ready = (
                not self.acknowledgement_topic
                or self.count_publishers(self.acknowledgement_topic) > 0
            )
            receipt_ready = (
                not self.receipt_topic
                or self.count_publishers(self.receipt_topic) > 0
            )
            if acknowledgement_ready and receipt_ready:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError("Timed out waiting for mapping acknowledgement publisher")

    def wait_for_receipt(
        self, previous_count: Optional[int], timeout_sec: float
    ) -> Optional[int]:
        if self.receipt_subscription is None:
            return None
        baseline = -1 if previous_count is None else previous_count
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.receipt_count is not None and self.receipt_count > baseline:
                return self.receipt_count
            rclpy.spin_once(
                self,
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())),
            )
        raise TimeoutError(
            "Mapping did not receive the published point cloud within "
            f"{timeout_sec:.1f} s (last receipt={self.receipt_count})."
        )

    def wait_for_direct_subscribers(self, timeout_sec: float = 20.0) -> None:
        if self.cloud_publisher is None:
            return
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.count_subscribers(CLOUD_TOPIC) > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise TimeoutError("Timed out waiting for mapping cloud subscriber")

    def wait_for_acknowledgement(
        self, previous_count: Optional[int], timeout_sec: float
    ) -> Optional[int]:
        if self.acknowledgement_subscription is None:
            return None
        baseline = -1 if previous_count is None else previous_count
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if (
                self.acknowledgement_count is not None
                and self.acknowledgement_count > baseline
            ):
                return self.acknowledgement_count
            rclpy.spin_once(
                self,
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())),
            )
        raise TimeoutError(
            "Mapping did not acknowledge the released point cloud within "
            f"{timeout_sec:.1f} s (last count={self.acknowledgement_count})."
        )

    def wait_for_player(self, process: subprocess.Popen, timeout_sec: float = 20.0) -> None:
        if self.burst_client is None:
            raise RuntimeError("rosbag2 burst client is disabled in direct mode")
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f"rosbag2 player exited with code {process.returncode}")
            if self.burst_client.wait_for_service(timeout_sec=0.2):
                return
        raise TimeoutError("Timed out waiting for /rosbag2_player/burst")

    def publish_static(self, transforms: Sequence[TransformStamped]) -> None:
        # StaticTransformBroadcaster does not accumulate separate calls.  Send
        # the full deduplicated tree in one call and keep this node alive.
        self.static_broadcaster.sendTransform(list(transforms))

    def prepare_stable_frame(self, frame: Frame) -> bool:
        if self.stable_inputs is None:
            raise RuntimeError("Stable input cache was not loaded")
        transforms, odometry, bracketed = self.stable_inputs.messages_for(frame.header_ns)
        if transforms:
            self.dynamic_broadcaster.sendTransform(transforms)
        if odometry is not None:
            self.odom_publisher.publish(odometry)
        return bracketed

    def burst(self, count: int, timeout_sec: float = 30.0) -> int:
        if self.burst_client is None:
            raise RuntimeError("rosbag2 burst client is disabled in direct mode")
        request = Burst.Request()
        request.num_messages = int(count)
        future = self.burst_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise TimeoutError("burst service did not return")
        return int(future.result().actually_burst)

    def publish_cloud(self, message: PointCloud2) -> None:
        if self.cloud_publisher is None:
            raise RuntimeError("Direct cloud publisher is not enabled")
        self.cloud_publisher.publish(message)


def start_player(layout: BagLayout, mode: str, clock_hz: float) -> subprocess.Popen:
    command = [
        "ros2",
        "bag",
        "play",
        str(layout.bag_dir),
        "--start-paused",
        "--disable-keyboard-controls",
        "--clock",
        str(clock_hz),
        "--log-level",
        "warn",
    ]
    if mode == "recorded":
        command.extend(["--remap"])
        command.extend(f"{source}:={target}" for source, target in RECORD_REMAPS.items())
    command.append("--topics")
    command.extend(MODE_TOPICS[mode])
    return subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )


def stop_player(process: Optional[subprocess.Popen]) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2.0)
    except ProcessLookupError:
        pass


def _format_time_ns(timestamp_ns: int) -> str:
    seconds = timestamp_ns / 1_000_000_000.0
    local = time.localtime(seconds)
    milliseconds = (timestamp_ns // 1_000_000) % 1000
    return time.strftime("%H:%M:%S", local) + f".{milliseconds:03d}"


def _read_key() -> str:
    if not sys.stdin.isatty():
        line = sys.stdin.readline()
        return "quit" if line == "" else "next"
    file_descriptor = sys.stdin.fileno()
    previous = termios.tcgetattr(file_descriptor)
    try:
        tty.setraw(file_descriptor)
        first = os.read(file_descriptor, 1)
        if first == b"\x03":
            raise KeyboardInterrupt
        if first in (b"q", b"Q", b"\x1b"):
            if first == b"\x1b" and select.select([file_descriptor], [], [], 0.04)[0]:
                suffix = os.read(file_descriptor, 2)
                if suffix == b"[C":
                    return "next"
            return "quit"
        if first in (b"\r", b"\n", b" ", b"n", b"N"):
            return "next"
        if first in (b"h", b"H", b"?"):
            return "help"
        return "ignore"
    finally:
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, previous)


def print_help(mode: str) -> None:
    anchor = MODE_ANCHOR[mode]
    print(
        "\n按键: Enter / 空格 / n / 右方向键 = 下一帧，q / Esc = 退出。\n"
        f"当前模式 {mode!r}，每帧锚点为 {anchor}。\n"
        "高程图是有状态的，不支持直接后退；要回退请重启 mapping 后从头播放。\n",
        flush=True,
    )


def parse_arguments() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Play one PointCloud2/map frame per key without modifying the bag."
    )
    parser.add_argument("bag", help="ROS 2 bag directory or .db3 path")
    parser.add_argument(
        "--mode",
        choices=sorted(MODE_TOPICS),
        default="stable",
        help=(
            "stable: prepublish bracketing TF then one cloud; faithful: preserve "
            "recorded input order; recorded: inspect stored filter maps"
        ),
    )
    parser.add_argument(
        "--index-dir",
        type=Path,
        default=script_dir / "indexes",
        help="Directory for lightweight JSON frame indexes",
    )
    parser.add_argument("--reindex", action="store_true", help="Rebuild cached index")
    parser.add_argument(
        "--prepare-only",
        action="store_true",
        help="Build/validate the index, then exit without publishing",
    )
    parser.add_argument(
        "--auto-steps",
        type=int,
        default=0,
        help="Advance this many frames non-interactively (used for testing)",
    )
    parser.add_argument(
        "--clock-hz", type=float, default=100.0, help="rosbag2 /clock frequency"
    )
    parser.add_argument(
        "--wait-after",
        type=float,
        default=0.25,
        help="Wall-clock wait after each frame so mapping/GPU work can finish",
    )
    parser.add_argument(
        "--ack-topic",
        default="",
        help=(
            "Optional std_msgs/UInt64 processed-cloud topic. When set, the next "
            "frame is not released until the mapper acknowledges the current one."
        ),
    )
    parser.add_argument(
        "--ack-timeout",
        type=float,
        default=30.0,
        help="Timeout in seconds for each mapping acknowledgement",
    )
    parser.add_argument(
        "--receipt-topic",
        default="",
        help=(
            "Optional std_msgs/UInt64 callback-entry topic used to detect and "
            "retry a point cloud lost before reaching the mapper."
        ),
    )
    parser.add_argument(
        "--receipt-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for callback entry before retrying direct publish",
    )
    parser.add_argument(
        "--direct-cloud-publish",
        action="store_true",
        help=(
            "Stable mode only: stream the original PointCloud2 rows through "
            "this long-lived publisher instead of using rosbag2 burst. This "
            "avoids first-message discovery loss."
        ),
    )
    arguments = parser.parse_args()
    if arguments.auto_steps < 0:
        parser.error("--auto-steps must be >= 0")
    if arguments.clock_hz <= 0.0:
        parser.error("--clock-hz must be > 0")
    if arguments.wait_after < 0.0:
        parser.error("--wait-after must be >= 0")
    if arguments.ack_timeout <= 0.0 or arguments.receipt_timeout <= 0.0:
        parser.error("--ack-timeout and --receipt-timeout must be > 0")
    if arguments.direct_cloud_publish and arguments.mode != "stable":
        parser.error("--direct-cloud-publish is only supported in stable mode")
    if arguments.direct_cloud_publish and not arguments.ack_topic:
        parser.error("--direct-cloud-publish requires --ack-topic")
    if arguments.direct_cloud_publish and not arguments.receipt_topic:
        parser.error("--direct-cloud-publish requires --receipt-topic")
    return arguments


def main() -> int:
    arguments = parse_arguments()
    layout = resolve_bag(arguments.bag)
    frames = load_or_build_frames(
        layout,
        arguments.mode,
        arguments.index_dir.resolve(),
        force=arguments.reindex,
    )
    print(
        f"索引完成: {layout.bag_dir.name}, mode={arguments.mode}, "
        f"{len(frames)} 帧, 原始 bag 未修改。",
        flush=True,
    )
    if arguments.prepare_only:
        print(index_path(arguments.index_dir.resolve(), layout, arguments.mode))
        return 0

    static_transforms = load_static_transforms(layout)
    stable_inputs = StableInputs(layout) if arguments.mode == "stable" else None
    cloud_rows = (
        iter(_serialized_rows(layout, CLOUD_TOPIC))
        if arguments.direct_cloud_publish
        else None
    )
    player: Optional[subprocess.Popen] = None
    node: Optional[StepController] = None
    try:
        rclpy.init()
        node = StepController(
            stable_inputs,
            arguments.ack_topic,
            arguments.receipt_topic,
            arguments.direct_cloud_publish,
        )
        if arguments.direct_cloud_publish:
            node.wait_for_direct_subscribers()
        else:
            player = start_player(layout, arguments.mode, arguments.clock_hz)
            node.wait_for_player(player)
        node.wait_for_acknowledgement_publisher(player)
        node.publish_static(static_transforms)
        # The point-cloud publisher lives in the separate rosbag2 process.
        # Give DDS discovery enough time on the vehicle computer before the
        # first large cloud is released; unmatched best-effort data is lost.
        time.sleep(2.0)
        print(
            f"已发布 {len(static_transforms)} 条去重静态 TF。"
            "请确认实车 FP、RSLidar 和旧静态 TF 脚本均已停止。",
            flush=True,
        )
        print_help(arguments.mode)

        frame_index = 0
        while frame_index < len(frames):
            if arguments.auto_steps:
                if frame_index >= arguments.auto_steps:
                    break
                command = "next"
            else:
                print(
                    f"[{frame_index + 1}/{len(frames)}] 等待按键 > ",
                    end="",
                    flush=True,
                )
                command = _read_key()
                print(flush=True)

            if command == "quit":
                break
            if command == "help":
                print_help(arguments.mode)
                continue
            if command != "next":
                continue

            frame = frames[frame_index]
            previous_acknowledgement = node.acknowledgement_count
            previous_receipt = node.receipt_count
            receipt = None
            bracketed = True
            if arguments.mode == "stable":
                bracketed = node.prepare_stable_frame(frame)
                # Give the mapper's separate TF listener time to receive both
                # sides of the interpolation bracket before releasing cloud.
                time.sleep(0.15)
                if arguments.direct_cloud_publish:
                    try:
                        cloud_record_ns, serialized_cloud = next(cloud_rows)
                    except StopIteration as exc:
                        raise RuntimeError(
                            "PointCloud2 stream ended before the frame index"
                        ) from exc
                    if cloud_record_ns != frame.record_ns:
                        raise RuntimeError(
                            "PointCloud2/index storage timestamp mismatch: "
                            f"{cloud_record_ns} != {frame.record_ns}"
                        )
                    cloud_message = deserialize_message(
                        serialized_cloud,
                        PointCloud2,
                    )
                    max_receipt_attempts = 10
                    for attempt in range(1, max_receipt_attempts + 1):
                        node.publish_cloud(cloud_message)
                        try:
                            receipt = node.wait_for_receipt(
                                previous_receipt,
                                arguments.receipt_timeout,
                            )
                            break
                        except TimeoutError:
                            if attempt == max_receipt_attempts:
                                raise
                            print(
                                "  cloud receipt timed out; retry "
                                f"{attempt + 1}/{max_receipt_attempts}",
                                flush=True,
                            )
                    released = 1
                else:
                    released = node.burst(1)
                success = released == 1
            else:
                released = node.burst(frame.burst_count)
                success = released == frame.burst_count

            lag_ms = (frame.record_ns - frame.header_ns) / 1_000_000.0
            if arguments.mode == "stable":
                tf_label = "TF_BRACKETED" if bracketed else "UNBRACKETED_TF"
            else:
                tf_label = "RECORDED_ORDER"
            print(
                f"frame={frame.number}/{len(frames)} "
                f"header={_format_time_ns(frame.header_ns)} "
                f"record-header={lag_ms:.1f} ms "
                f"released={released}/{frame.burst_count} {tf_label}",
                flush=True,
            )
            if not success:
                print("播放器已到结尾或未释放完整帧。", file=sys.stderr)
                break
            acknowledgement = node.wait_for_acknowledgement(
                previous_acknowledgement,
                arguments.ack_timeout,
            )
            if acknowledgement is not None:
                receipt_text = "" if receipt is None else f" receipt={receipt}"
                print(
                    f"  mapping_ack={acknowledgement}{receipt_text}",
                    flush=True,
                )
            frame_index += 1
            if arguments.wait_after:
                time.sleep(arguments.wait_after)

        print(f"已播放 {frame_index}/{len(frames)} 帧。", flush=True)
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在停止播放器……", flush=True)
        return 130
    finally:
        stop_player(player)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
