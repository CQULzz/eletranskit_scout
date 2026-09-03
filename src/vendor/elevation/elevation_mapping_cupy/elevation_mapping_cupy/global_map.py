"""Sparse, tiled, disk-backed global elevation-map storage.

The CUDA mapper intentionally remains a small rolling window. Cells updated by
each point cloud are fused, with their variance, into fixed-size CPU tiles
keyed by integer map coordinates. Only observed tiles exist, so a long narrow
route does not allocate the empty rectangle around it. A bounded LRU cache
keeps hot tiles in RAM while a background SQLite writer persists cold tiles.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import queue
import sqlite3
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple
import zlib

import numpy as np


TileKey = Tuple[int, int]


@dataclass(frozen=True)
class GlobalMapFuseResult:
    """Statistics returned by one local-to-global fusion."""

    valid_input_cells: int
    written_cells: int
    touched_tiles: int
    rejected_cells: int = 0


@dataclass(frozen=True)
class GlobalMapWindow:
    """Finite dense view extracted from the unbounded tiled map."""

    elevation_yx: np.ndarray
    variance_yx: np.ndarray
    observation_count_yx: np.ndarray
    last_update_revision_yx: np.ndarray
    resolution: float
    center_x: float
    center_y: float
    length_x: float
    length_y: float

    def elevation_grid_map_copy(self) -> np.ndarray:
        """Convert natural ``row=Y, col=X`` storage to GridMap coordinates.

        The mapper's rolling-map publisher performs the same transformation:
        GridMap assigns its first matrix axis to decreasing X and its second
        axis to decreasing Y.  The transpose swaps the natural Y/X axes, and
        the two flips align their signs.  Column-major ROS serialization is a
        separate operation performed by ``encode_layer_to_multiarray``.
        """

        return np.ascontiguousarray(
            np.flip(self.elevation_yx.T, axis=(0, 1))
        )

    def layer_grid_map_copy(self, layer: str) -> np.ndarray:
        """Convert a named natural-layout global layer to GridMap layout."""

        arrays = {
            "elevation": self.elevation_yx,
            "variance": self.variance_yx,
            "observation_count": self.observation_count_yx,
            "last_update_revision": self.last_update_revision_yx,
        }
        if layer not in arrays:
            raise KeyError(f"Unknown global-map layer: {layer}")
        return np.ascontiguousarray(np.flip(arrays[layer].T, axis=(0, 1)))


@dataclass
class _TileEntry:
    data: np.ndarray
    revision: int
    persisted_revision: int


def downsample_elevation_blocks(
    elevation_yx: np.ndarray, stride: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return one representative point for every non-empty stride block.

    X/Y are returned as fractional cell offsets from the tile's first cell
    center. Heights are finite-value means. This preserves thin observed strips
    that fixed-lattice slicing would miss and places each marker at its block
    center instead of a corner.
    """

    tile = np.asarray(elevation_yx, dtype=np.float32)
    if tile.ndim != 2:
        raise ValueError(f"Elevation tile must be 2-D, got {tile.shape}.")
    if int(stride) != stride or stride <= 0:
        raise ValueError("Visualization stride must be a positive integer.")
    stride = int(stride)
    rows, cols = tile.shape
    if rows == 0 or cols == 0:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty

    block_rows = (rows + stride - 1) // stride
    block_cols = (cols + stride - 1) // stride
    padded = np.full(
        (block_rows * stride, block_cols * stride), np.nan, dtype=np.float32
    )
    padded[:rows, :cols] = tile
    blocks = padded.reshape(block_rows, stride, block_cols, stride).transpose(
        0, 2, 1, 3
    )
    finite = np.isfinite(blocks)
    counts = finite.sum(axis=(2, 3))
    sums = np.where(finite, blocks, 0.0).sum(axis=(2, 3), dtype=np.float64)
    means = np.full((block_rows, block_cols), np.nan, dtype=np.float32)
    np.divide(sums, counts, out=means, where=counts > 0, casting="unsafe")

    block_y, block_x = np.nonzero(counts > 0)
    width = np.minimum(stride, cols - block_x * stride)
    height = np.minimum(stride, rows - block_y * stride)
    center_x = block_x * stride + 0.5 * (width - 1)
    center_y = block_y * stride + 0.5 * (height - 1)
    return (
        center_x.astype(np.float32, copy=False),
        center_y.astype(np.float32, copy=False),
        means[block_y, block_x],
    )


class _SqliteTileBackend:
    """SQLite persistence with compression on a dedicated writer thread."""

    # Version 2 stores four float32 layers per tile: elevation, variance,
    # accepted observation count, and last-update revision.
    _SCHEMA_VERSION = 2

    def __init__(
        self,
        path: str,
        *,
        resolution: float,
        tile_size_cells: int,
        map_frame: str,
        resume: bool,
        max_pending_tiles: int,
        persisted_callback,
    ) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._resolution = float(resolution)
        self._tile_size_cells = int(tile_size_cells)
        self._map_frame = str(map_frame)
        self._persisted_callback = persisted_callback
        self._max_pending_tiles = int(max_pending_tiles)
        self._error_lock = threading.Lock()
        self._error: Optional[BaseException] = None
        self._closed = False
        self._pending_lock = threading.Lock()
        self._accepting_writes = True
        self._pending_tiles: Dict[TileKey, Tuple[int, np.ndarray]] = {}
        self._scheduled_tiles = set()
        self._pending_metadata: Optional[Dict[str, object]] = None
        self._metadata_scheduled = False
        self._read_lock = threading.RLock()

        self._read_conn = self._connect()
        try:
            self._create_schema(self._read_conn)
            metadata = self.read_metadata()
            if resume:
                if metadata:
                    self._validate_metadata(metadata)
                else:
                    tile_count = self._read_conn.execute(
                        "SELECT COUNT(*) FROM tiles"
                    ).fetchone()[0]
                    if tile_count:
                        raise ValueError(
                            "Global tile database contains tiles but no geometry metadata."
                        )
            else:
                self._read_conn.execute("DELETE FROM tiles")
                self._read_conn.execute("DELETE FROM metadata")
                self._read_conn.commit()
        except BaseException:
            self._read_conn.close()
            raise

        # The queue contains only one small token per tile key. Repeated
        # updates are coalesced in _pending_tiles, so a slow disk cannot create
        # an unbounded queue of full tile snapshots.
        self._queue: queue.Queue = queue.Queue()
        self._writer = threading.Thread(
            target=self._writer_main,
            name="global_elevation_sqlite_writer",
            daemon=True,
        )
        self._writer.start()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(
            str(self.path), timeout=30.0, check_same_thread=False
        )
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA busy_timeout=30000")
        return conn

    @staticmethod
    def _create_schema(conn: sqlite3.Connection) -> None:
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS metadata (
                key TEXT PRIMARY KEY,
                value TEXT NOT NULL
            )
            """
        )
        conn.execute(
            """
            CREATE TABLE IF NOT EXISTS tiles (
                level INTEGER NOT NULL,
                tile_x INTEGER NOT NULL,
                tile_y INTEGER NOT NULL,
                revision INTEGER NOT NULL,
                payload BLOB NOT NULL,
                PRIMARY KEY(level, tile_x, tile_y)
            ) WITHOUT ROWID
            """
        )
        conn.commit()

    def _validate_metadata(self, metadata: Dict[str, object]) -> None:
        expected = {
            "schema_version": self._SCHEMA_VERSION,
            "resolution": self._resolution,
            "tile_size_cells": self._tile_size_cells,
            "map_frame": self._map_frame,
        }
        for key, value in expected.items():
            actual = metadata.get(key)
            if isinstance(value, float):
                matches = actual is not None and math.isclose(
                    float(actual), value, rel_tol=0.0, abs_tol=1.0e-9
                )
            else:
                matches = actual == value
            if not matches:
                raise ValueError(
                    f"Global tile database metadata mismatch for '{key}': "
                    f"database={actual!r}, requested={value!r}."
                )

    def read_metadata(self) -> Dict[str, object]:
        with self._read_lock:
            rows = self._read_conn.execute(
                "SELECT key, value FROM metadata"
            ).fetchall()
        return {key: json.loads(value) for key, value in rows}

    def load_tile(self, key: TileKey, tile_shape: Tuple[int, int]):
        with self._read_lock:
            row = self._read_conn.execute(
                "SELECT revision, payload FROM tiles "
                "WHERE level=0 AND tile_x=? AND tile_y=?",
                (int(key[0]), int(key[1])),
            ).fetchone()
        if row is None:
            return None
        revision, payload = row
        raw = zlib.decompress(payload)
        expected_bytes = int(np.prod(tile_shape)) * np.dtype("<f4").itemsize
        if len(raw) != expected_bytes:
            raise ValueError(
                f"Corrupt global tile {key}: got {len(raw)} bytes, expected {expected_bytes}."
            )
        data = np.frombuffer(raw, dtype="<f4").reshape(tile_shape).astype(
            np.float32, copy=True
        )
        return int(revision), data

    def all_keys(self) -> List[TileKey]:
        with self._read_lock:
            rows = self._read_conn.execute(
                "SELECT tile_x, tile_y FROM tiles WHERE level=0"
            ).fetchall()
        return [(int(tile_x), int(tile_y)) for tile_x, tile_y in rows]

    def max_revision(self) -> int:
        with self._read_lock:
            row = self._read_conn.execute(
                "SELECT COALESCE(MAX(revision), 0) FROM tiles WHERE level=0"
            ).fetchone()
        return int(row[0])

    def enqueue_tile(self, key: TileKey, revision: int, data: np.ndarray) -> None:
        self.raise_if_error()
        snapshot = np.ascontiguousarray(data)
        with self._pending_lock:
            if not self._accepting_writes:
                raise RuntimeError("Global-map SQLite backend is stopping.")
            schedule = key not in self._scheduled_tiles
            if schedule and len(self._scheduled_tiles) >= self._max_pending_tiles:
                raise RuntimeError(
                    "Global-map persistence cannot keep up: "
                    f"{self._max_pending_tiles} distinct tiles are already pending."
                )
            self._pending_tiles[key] = (int(revision), snapshot)
            if schedule:
                self._scheduled_tiles.add(key)
                self._queue.put(("tile_key", key))

    def enqueue_metadata(self, metadata: Dict[str, object]) -> None:
        self.raise_if_error()
        with self._pending_lock:
            if not self._accepting_writes:
                raise RuntimeError("Global-map SQLite backend is stopping.")
            self._pending_metadata = dict(metadata)
            if not self._metadata_scheduled:
                self._metadata_scheduled = True
                self._queue.put(("metadata",))

    def wait_idle(self) -> None:
        self._wait_for_control("barrier")

    def clear(self) -> None:
        self._wait_for_control("clear")

    def raise_if_error(self) -> None:
        with self._error_lock:
            error = self._error
        if error is not None:
            raise RuntimeError(f"Global-map SQLite writer failed: {error}") from error

    def close(self) -> None:
        if self._closed:
            return
        failure: Optional[BaseException] = None
        try:
            if self._writer.is_alive():
                with self._pending_lock:
                    self._accepting_writes = False
                self._wait_for_control("stop")
            else:
                self.raise_if_error()
                raise RuntimeError("Global-map SQLite writer stopped unexpectedly.")
        except BaseException as exc:
            failure = exc
        finally:
            self._writer.join(timeout=5.0)
            self._read_conn.close()
            self._closed = True
        if self._writer.is_alive() and failure is None:
            failure = RuntimeError("Global-map SQLite writer did not stop.")
        if failure is not None:
            raise failure

    def _wait_for_control(self, command: str) -> None:
        """Wait for a writer barrier without hanging after a writer failure."""

        self.raise_if_error()
        if self._closed:
            raise RuntimeError("Global-map SQLite backend is closed.")
        if not self._writer.is_alive():
            self.raise_if_error()
            raise RuntimeError("Global-map SQLite writer stopped unexpectedly.")

        event = threading.Event()
        self._queue.put((command, event))
        deadline = time.monotonic() + 45.0
        while not event.wait(timeout=0.1):
            if not self._writer.is_alive():
                self.raise_if_error()
                raise RuntimeError("Global-map SQLite writer stopped unexpectedly.")
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"Timed out waiting for global-map SQLite '{command}' operation."
                )
        self.raise_if_error()

    def _writer_main(self) -> None:
        conn = None
        current_batch = []
        try:
            conn = self._connect()
            self._create_schema(conn)
            should_stop = False
            while not should_stop:
                first = self._queue.get()
                current_batch = [first]
                for _ in range(63):
                    try:
                        current_batch.append(self._queue.get_nowait())
                    except queue.Empty:
                        break

                persisted = []
                controls = []
                with conn:
                    for item in current_batch:
                        if item[0] == "tile_key":
                            key = item[1]
                            with self._pending_lock:
                                pending = self._pending_tiles.pop(key, None)
                                self._scheduled_tiles.discard(key)
                            if pending is None:
                                continue
                            revision, data = pending
                            raw = np.asarray(
                                data, dtype="<f4", order="C"
                            ).tobytes(order="C")
                            payload = sqlite3.Binary(zlib.compress(raw, level=1))
                            conn.execute(
                                "INSERT INTO tiles"
                                "(level, tile_x, tile_y, revision, payload) "
                                "VALUES (?, ?, ?, ?, ?) "
                                "ON CONFLICT(level, tile_x, tile_y) DO UPDATE SET "
                                "revision=excluded.revision, payload=excluded.payload "
                                "WHERE excluded.revision >= tiles.revision",
                                (
                                    0,
                                    int(key[0]),
                                    int(key[1]),
                                    int(revision),
                                    payload,
                                ),
                            )
                            persisted.append((key, revision))
                        elif item[0] == "metadata":
                            with self._pending_lock:
                                metadata = self._pending_metadata
                                self._pending_metadata = None
                                self._metadata_scheduled = False
                            if metadata is None:
                                continue
                            rows = [
                                (key, json.dumps(value))
                                for key, value in metadata.items()
                            ]
                            conn.executemany(
                                "INSERT INTO metadata(key, value) VALUES (?, ?) "
                                "ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                                rows,
                            )
                        elif item[0] == "clear":
                            conn.execute("DELETE FROM tiles")
                            conn.execute("DELETE FROM metadata")
                            controls.append(item)
                        elif item[0] in {"barrier", "stop"}:
                            if item[0] == "stop":
                                should_stop = True
                            controls.append(item)
                        else:
                            raise RuntimeError(
                                f"Unknown SQLite writer command: {item[0]!r}."
                            )

                for key, revision in persisted:
                    self._persisted_callback(key, revision)
                for control in controls:
                    control[1].set()
                for _ in current_batch:
                    self._queue.task_done()
                current_batch = []
        except BaseException as exc:
            with self._error_lock:
                self._error = exc
            for item in current_batch:
                if item[0] in {"barrier", "clear", "stop"}:
                    item[1].set()
                self._queue.task_done()
            while True:
                try:
                    item = self._queue.get_nowait()
                except queue.Empty:
                    break
                if item[0] in {"barrier", "clear", "stop"}:
                    item[1].set()
                self._queue.task_done()
        finally:
            if conn is not None:
                conn.close()


class GlobalMapAccumulator:
    """Logically unbounded sparse elevation grid backed by fixed-size tiles."""

    # The rolling center is float32 and can drift by about 1e-3 cell after
    # repeated shifts. Keep enough margin for that numerical error while still
    # rejecting a real sub-cell geometry mismatch.
    _CENTER_SNAP_TOLERANCE_CELLS = 0.05
    _ELEVATION = 0
    _VARIANCE = 1
    _OBSERVATION_COUNT = 2
    _LAST_UPDATE_REVISION = 3
    _LAYER_COUNT = 4

    def __init__(
        self,
        *,
        resolution: float,
        tile_size_cells: int = 128,
        max_tiles_in_memory: int = 256,
        storage_path: str = "",
        resume_from_disk: bool = False,
        map_frame: str = "map",
        variance_floor: float = 0.0025,
        variance_ceiling: float = 4.0,
        outlier_sigma: float = 3.0,
        max_height_jump: float = 0.30,
    ) -> None:
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError("Global-map resolution must be finite and positive.")
        if int(tile_size_cells) != tile_size_cells or tile_size_cells <= 0:
            raise ValueError("global_map_tile_size_cells must be a positive integer.")
        if int(max_tiles_in_memory) != max_tiles_in_memory or max_tiles_in_memory <= 0:
            raise ValueError("global_map_max_tiles_in_memory must be a positive integer.")
        if not map_frame:
            raise ValueError("Global-map frame must be non-empty.")
        if not math.isfinite(variance_floor) or variance_floor <= 0.0:
            raise ValueError("Global-map variance floor must be finite and positive.")
        if (
            not math.isfinite(variance_ceiling)
            or variance_ceiling < variance_floor
        ):
            raise ValueError(
                "Global-map variance ceiling must be finite and no smaller than the floor."
            )
        if not math.isfinite(outlier_sigma) or outlier_sigma <= 0.0:
            raise ValueError("Global-map outlier sigma must be finite and positive.")
        if not math.isfinite(max_height_jump) or max_height_jump <= 0.0:
            raise ValueError("Global-map maximum height jump must be finite and positive.")

        self.resolution = float(resolution)
        self.tile_size_cells = int(tile_size_cells)
        self.max_tiles_in_memory = int(max_tiles_in_memory)
        self.map_frame = str(map_frame)
        self.variance_floor = float(variance_floor)
        self.variance_ceiling = float(variance_ceiling)
        self.outlier_sigma = float(outlier_sigma)
        self.max_height_jump = float(max_height_jump)
        self._spatial_tile_shape = (self.tile_size_cells, self.tile_size_cells)
        self._tile_shape = (
            self._LAYER_COUNT,
            self.tile_size_cells,
            self.tile_size_cells,
        )
        # Serializes public operations with clear/close. The SQLite callback
        # intentionally uses only _lock, so waiting for the writer cannot
        # deadlock on this lifecycle lock.
        self._operation_lock = threading.RLock()
        self._state = "open"
        self._lock = threading.RLock()
        self._tiles: "OrderedDict[TileKey, _TileEntry]" = OrderedDict()
        self._pending: Dict[TileKey, Tuple[int, np.ndarray]] = {}
        self._visualization_dirty: "OrderedDict[TileKey, None]" = OrderedDict()
        self._revision = 0
        self._phase_x: Optional[float] = None
        self._phase_y: Optional[float] = None
        self._min_ix: Optional[int] = None
        self._max_ix: Optional[int] = None
        self._min_iy: Optional[int] = None
        self._max_iy: Optional[int] = None

        self._backend: Optional[_SqliteTileBackend] = None
        if storage_path:
            self._backend = _SqliteTileBackend(
                storage_path,
                resolution=self.resolution,
                tile_size_cells=self.tile_size_cells,
                map_frame=self.map_frame,
                resume=bool(resume_from_disk),
                max_pending_tiles=max(16, self.max_tiles_in_memory * 2),
                persisted_callback=self._on_tile_persisted,
            )
            try:
                if resume_from_disk:
                    metadata = self._backend.read_metadata()
                    self._load_state_metadata(metadata)
                    self._revision = max(
                        self._revision, self._backend.max_revision()
                    )
                    persisted_keys = self._backend.all_keys()
                    if persisted_keys and self._min_ix is None:
                        tile_x = [key[0] for key in persisted_keys]
                        tile_y = [key[1] for key in persisted_keys]
                        self._min_ix = min(tile_x) * self.tile_size_cells
                        self._max_ix = (
                            (max(tile_x) + 1) * self.tile_size_cells - 1
                        )
                        self._min_iy = min(tile_y) * self.tile_size_cells
                        self._max_iy = (
                            (max(tile_y) + 1) * self.tile_size_cells - 1
                        )
                    elif not persisted_keys:
                        self._min_ix = self._max_ix = None
                        self._min_iy = self._max_iy = None
            except BaseException:
                self._backend.close()
                self._backend = None
                raise

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    @property
    def has_observations(self) -> bool:
        with self._lock:
            return self._min_ix is not None

    @property
    def phase_x(self) -> float:
        if self._phase_x is None:
            raise RuntimeError("Global-map lattice has not been initialized.")
        return self._phase_x

    @property
    def phase_y(self) -> float:
        if self._phase_y is None:
            raise RuntimeError("Global-map lattice has not been initialized.")
        return self._phase_y

    @property
    def storage_path(self) -> str:
        return str(self._backend.path) if self._backend is not None else ""

    @property
    def cached_tile_count(self) -> int:
        with self._lock:
            return len(self._tiles)

    def tile_count(self) -> int:
        return len(self.all_tile_keys())

    def initialize_lattice(self, shape_yx: Sequence[int]) -> None:
        """Set the cell-center phase from the fixed local-map shape."""

        with self._operation_lock:
            self._ensure_open()
            if len(shape_yx) != 2:
                raise ValueError(
                    f"Local snapshot shape must have two axes, got {shape_yx!r}."
                )
            cells_y, cells_x = int(shape_yx[0]), int(shape_yx[1])
            if cells_x <= 0 or cells_y <= 0:
                raise ValueError(
                    f"Local snapshot shape must be positive, got {shape_yx!r}."
                )
            phase_x = 0.5 if cells_x % 2 == 0 else 0.0
            phase_y = 0.5 if cells_y % 2 == 0 else 0.0
            with self._lock:
                if self._phase_x is not None and not math.isclose(
                    self._phase_x, phase_x
                ):
                    raise ValueError(
                        "Local X cell parity does not match the persisted global lattice."
                    )
                if self._phase_y is not None and not math.isclose(
                    self._phase_y, phase_y
                ):
                    raise ValueError(
                        "Local Y cell parity does not match the persisted global lattice."
                    )
                self._phase_x = phase_x
                self._phase_y = phase_y

    def validate_snapshot_geometry(
        self,
        center_xy: Sequence[float],
        shape_yx: Sequence[int],
    ) -> Tuple[int, int]:
        """Return integer indices of the local snapshot's lower-left cell."""

        with self._operation_lock:
            self._ensure_open()
            return self._validate_snapshot_geometry(center_xy, shape_yx)

    def _validate_snapshot_geometry(
        self,
        center_xy: Sequence[float],
        shape_yx: Sequence[int],
    ) -> Tuple[int, int]:

        self.initialize_lattice(shape_yx)
        center = np.asarray(center_xy, dtype=np.float64)
        if center.shape != (2,) or not np.isfinite(center).all():
            raise ValueError(
                f"Local snapshot center must contain two finite values, got {center_xy!r}."
            )
        center_cells = center / self.resolution
        snapped = np.rint(center_cells).astype(np.int64)
        error = np.abs(center_cells - snapped)
        if np.any(error > self._CENTER_SNAP_TOLERANCE_CELLS):
            raise ValueError(
                "Local map center is too far from the global integer lattice: "
                f"center={tuple(center)}, error_cells={tuple(error)}."
            )
        cells_y, cells_x = int(shape_yx[0]), int(shape_yx[1])
        start_ix = int(snapped[0]) - cells_x // 2
        start_iy = int(snapped[1]) - cells_y // 2
        return start_ix, start_iy

    def fuse_snapshot(
        self,
        center_xy: Sequence[float],
        elevation_yx: np.ndarray,
        variance_yx: Optional[np.ndarray] = None,
        updated_yx: Optional[np.ndarray] = None,
    ) -> GlobalMapFuseResult:
        """Fuse newly observed local cells into probabilistic global tiles."""

        with self._operation_lock:
            self._ensure_open()
            return self._fuse_snapshot(
                center_xy, elevation_yx, variance_yx, updated_yx
            )

    def _fuse_snapshot(
        self,
        center_xy: Sequence[float],
        elevation_yx: np.ndarray,
        variance_yx: Optional[np.ndarray],
        updated_yx: Optional[np.ndarray],
    ) -> GlobalMapFuseResult:

        local = np.asarray(elevation_yx, dtype=np.float32)
        if local.ndim != 2:
            raise ValueError(f"Local elevation must be 2-D, got {local.shape}.")
        if variance_yx is None:
            variance = np.ones(local.shape, dtype=np.float32)
        else:
            variance = np.asarray(variance_yx, dtype=np.float32)
            if variance.shape != local.shape:
                raise ValueError(
                    "Local variance shape must match elevation: "
                    f"{variance.shape} vs {local.shape}."
                )
        if updated_yx is None:
            updated = np.ones(local.shape, dtype=bool)
        else:
            updated = np.asarray(updated_yx, dtype=bool)
            if updated.shape != local.shape:
                raise ValueError(
                    "Local update-mask shape must match elevation: "
                    f"{updated.shape} vs {local.shape}."
                )
        start_ix, start_iy = self._validate_snapshot_geometry(center_xy, local.shape)
        valid_input = (
            updated
            & np.isfinite(local)
            & np.isfinite(variance)
            & (variance > 0.0)
        )
        valid_input_cells = int(valid_input.sum())
        if valid_input_cells == 0:
            return GlobalMapFuseResult(0, 0, 0)

        variance = np.clip(
            variance, self.variance_floor, self.variance_ceiling
        )

        cells_y, cells_x = local.shape
        end_ix = start_ix + cells_x
        end_iy = start_iy + cells_y
        min_tx = start_ix // self.tile_size_cells
        max_tx = (end_ix - 1) // self.tile_size_cells
        min_ty = start_iy // self.tile_size_cells
        max_ty = (end_iy - 1) // self.tile_size_cells
        written_cells = 0
        rejected_cells = 0
        touched: List[TileKey] = []

        with self._lock:
            self._raise_backend_error_locked()
            self._revision += 1
            revision = self._revision
            for tile_y in range(min_ty, max_ty + 1):
                tile_iy0 = tile_y * self.tile_size_cells
                overlap_iy0 = max(start_iy, tile_iy0)
                overlap_iy1 = min(end_iy, tile_iy0 + self.tile_size_cells)
                source_y0 = overlap_iy0 - start_iy
                dest_y0 = overlap_iy0 - tile_iy0
                height = overlap_iy1 - overlap_iy0
                for tile_x in range(min_tx, max_tx + 1):
                    tile_ix0 = tile_x * self.tile_size_cells
                    overlap_ix0 = max(start_ix, tile_ix0)
                    overlap_ix1 = min(end_ix, tile_ix0 + self.tile_size_cells)
                    source_x0 = overlap_ix0 - start_ix
                    dest_x0 = overlap_ix0 - tile_ix0
                    width = overlap_ix1 - overlap_ix0
                    source_height = local[
                        source_y0 : source_y0 + height,
                        source_x0 : source_x0 + width,
                    ]
                    source_variance = variance[
                        source_y0 : source_y0 + height,
                        source_x0 : source_x0 + width,
                    ]
                    valid = valid_input[
                        source_y0 : source_y0 + height,
                        source_x0 : source_x0 + width,
                    ]
                    count = int(valid.sum())
                    if count == 0:
                        continue

                    key = (tile_x, tile_y)
                    entry = self._get_tile_locked(key, create=True)
                    destination = entry.data[
                        :,
                        dest_y0 : dest_y0 + height,
                        dest_x0 : dest_x0 + width,
                    ]
                    destination_height = destination[self._ELEVATION]
                    destination_variance = destination[self._VARIANCE]
                    destination_count = destination[self._OBSERVATION_COUNT]
                    destination_revision = destination[self._LAST_UPDATE_REVISION]

                    unseen = valid & (
                        ~np.isfinite(destination_height)
                        | ~np.isfinite(destination_variance)
                        | (destination_count <= 0.0)
                    )
                    destination_height[unseen] = source_height[unseen]
                    destination_variance[unseen] = source_variance[unseen]
                    destination_count[unseen] = 1.0
                    destination_revision[unseen] = float(revision)

                    existing = valid & ~unseen
                    innovation = source_height - destination_height
                    combined_variance = destination_variance + source_variance
                    statistical_limit = self.outlier_sigma * np.sqrt(
                        np.maximum(combined_variance, self.variance_floor)
                    )
                    accepted_existing = existing & (
                        np.abs(innovation)
                        <= np.minimum(statistical_limit, self.max_height_jump)
                    )
                    if accepted_existing.any():
                        gain = np.zeros_like(source_height, dtype=np.float32)
                        denominator = combined_variance[accepted_existing]
                        gain[accepted_existing] = (
                            destination_variance[accepted_existing] / denominator
                        )
                        destination_height[accepted_existing] += (
                            gain[accepted_existing] * innovation[accepted_existing]
                        )
                        destination_variance[accepted_existing] = np.maximum(
                            self.variance_floor,
                            (1.0 - gain[accepted_existing])
                            * destination_variance[accepted_existing],
                        )
                        destination_count[accepted_existing] = np.minimum(
                            destination_count[accepted_existing] + 1.0,
                            np.float32(16_777_216.0),
                        )
                        destination_revision[accepted_existing] = float(revision)

                    rejected = existing & ~accepted_existing
                    accepted = unseen | accepted_existing
                    accepted_count = int(accepted.sum())
                    rejected_cells += int(rejected.sum())
                    if accepted_count == 0:
                        continue
                    entry.revision = revision
                    self._tiles.move_to_end(key)
                    self._mark_visualization_dirty_locked(key)
                    touched.append(key)
                    written_cells += accepted_count

                    valid_y, valid_x = np.nonzero(accepted)
                    self._extend_bounds_locked(
                        overlap_ix0 + int(valid_x.min()),
                        overlap_ix0 + int(valid_x.max()),
                        overlap_iy0 + int(valid_y.min()),
                        overlap_iy0 + int(valid_y.max()),
                    )
            self._evict_if_needed_locked()

        return GlobalMapFuseResult(
            valid_input_cells=valid_input_cells,
            written_cells=written_cells,
            touched_tiles=len(set(touched)),
            rejected_cells=rejected_cells,
        )

    def extract_window(
        self,
        center_xy: Sequence[float],
        length_x: float,
        length_y: float,
    ) -> GlobalMapWindow:
        """Materialize a finite dense ROI for GridMap/RViz consumers."""

        with self._operation_lock:
            self._ensure_open()
            return self._extract_window(center_xy, length_x, length_y)

    def _extract_window(
        self,
        center_xy: Sequence[float],
        length_x: float,
        length_y: float,
    ) -> GlobalMapWindow:

        center = np.asarray(center_xy, dtype=np.float64)
        if center.shape != (2,) or not np.isfinite(center).all():
            raise ValueError("Global-map view center must contain two finite values.")
        if not math.isfinite(length_x) or not math.isfinite(length_y):
            raise ValueError("Global-map view lengths must be finite.")
        cells_x = int(round(length_x / self.resolution))
        cells_y = int(round(length_y / self.resolution))
        if cells_x <= 0 or cells_y <= 0:
            raise ValueError("Global-map view lengths must be positive.")

        with self._lock:
            if self._phase_x is None or self._phase_y is None:
                raise RuntimeError("Global-map lattice has not been initialized.")
            expected_x_even = math.isclose(self._phase_x, 0.5)
            expected_y_even = math.isclose(self._phase_y, 0.5)
            if (cells_x % 2 == 0) != expected_x_even:
                cells_x += 1
            if (cells_y % 2 == 0) != expected_y_even:
                cells_y += 1

            center_ix = int(round(center[0] / self.resolution))
            center_iy = int(round(center[1] / self.resolution))
            start_ix = center_ix - cells_x // 2
            start_iy = center_iy - cells_y // 2
            output = np.full(
                (self._LAYER_COUNT, cells_y, cells_x),
                np.nan,
                dtype=np.float32,
            )
            output[self._OBSERVATION_COUNT].fill(0.0)
            output[self._LAST_UPDATE_REVISION].fill(0.0)
            end_ix = start_ix + cells_x
            end_iy = start_iy + cells_y
            min_tx = start_ix // self.tile_size_cells
            max_tx = (end_ix - 1) // self.tile_size_cells
            min_ty = start_iy // self.tile_size_cells
            max_ty = (end_iy - 1) // self.tile_size_cells

            for tile_y in range(min_ty, max_ty + 1):
                tile_iy0 = tile_y * self.tile_size_cells
                overlap_iy0 = max(start_iy, tile_iy0)
                overlap_iy1 = min(end_iy, tile_iy0 + self.tile_size_cells)
                for tile_x in range(min_tx, max_tx + 1):
                    key = (tile_x, tile_y)
                    entry = self._get_tile_locked(key, create=False)
                    if entry is None:
                        continue
                    tile_ix0 = tile_x * self.tile_size_cells
                    overlap_ix0 = max(start_ix, tile_ix0)
                    overlap_ix1 = min(end_ix, tile_ix0 + self.tile_size_cells)
                    source_y0 = overlap_iy0 - tile_iy0
                    source_x0 = overlap_ix0 - tile_ix0
                    dest_y0 = overlap_iy0 - start_iy
                    dest_x0 = overlap_ix0 - start_ix
                    height = overlap_iy1 - overlap_iy0
                    width = overlap_ix1 - overlap_ix0
                    output[
                        :,
                        dest_y0 : dest_y0 + height,
                        dest_x0 : dest_x0 + width,
                    ] = entry.data[
                        :,
                        source_y0 : source_y0 + height,
                        source_x0 : source_x0 + width,
                    ]
                    self._evict_if_needed_locked()
            self._evict_if_needed_locked()

            actual_center_x = (
                start_ix + self._phase_x - 0.5 + 0.5 * cells_x
            ) * self.resolution
            actual_center_y = (
                start_iy + self._phase_y - 0.5 + 0.5 * cells_y
            ) * self.resolution
            return GlobalMapWindow(
                elevation_yx=output[self._ELEVATION],
                variance_yx=output[self._VARIANCE],
                observation_count_yx=output[self._OBSERVATION_COUNT],
                last_update_revision_yx=output[self._LAST_UPDATE_REVISION],
                resolution=self.resolution,
                center_x=float(actual_center_x),
                center_y=float(actual_center_y),
                length_x=cells_x * self.resolution,
                length_y=cells_y * self.resolution,
            )

    def all_tile_keys(self) -> List[TileKey]:
        with self._operation_lock:
            self._ensure_open()
            return self._all_tile_keys()

    def _all_tile_keys(self) -> List[TileKey]:
        with self._lock:
            keys = set(self._tiles.keys()) | set(self._pending.keys())
            if self._backend is not None:
                keys.update(self._backend.all_keys())
            return sorted(keys, key=lambda key: (key[1], key[0]))

    def queue_all_tiles_for_visualization(self) -> None:
        with self._operation_lock:
            self._ensure_open()
            with self._lock:
                for key in self._all_tile_keys():
                    self._mark_visualization_dirty_locked(key)

    def discard_visualization_updates(self) -> None:
        """Drop incremental RViz work; a later subscriber requests a replay."""

        with self._operation_lock:
            self._ensure_open()
            with self._lock:
                self._visualization_dirty.clear()

    def take_visualization_tiles(
        self, max_tiles: int
    ) -> List[Tuple[TileKey, np.ndarray]]:
        with self._operation_lock:
            self._ensure_open()
            if max_tiles <= 0:
                raise ValueError("max_tiles must be positive.")
            result = []
            with self._lock:
                for _ in range(min(max_tiles, len(self._visualization_dirty))):
                    key, _ = self._visualization_dirty.popitem(last=False)
                    entry = self._get_tile_locked(key, create=False)
                    if entry is not None:
                        result.append(
                            (key, entry.data[self._ELEVATION].copy())
                        )
                    self._evict_if_needed_locked()
            return result

    def flush(self, wait: bool = False) -> None:
        with self._operation_lock:
            self._ensure_open()
            self._flush(wait=wait)

    def _flush(self, wait: bool = False) -> None:
        if self._backend is None:
            return
        with self._lock:
            self._raise_backend_error_locked()
            for key, entry in list(self._tiles.items()):
                if entry.revision > entry.persisted_revision:
                    self._enqueue_tile_locked(key, entry)
            self._backend.enqueue_metadata(self._state_metadata_locked())
        if wait:
            self._backend.wait_idle()

    def clear(self) -> None:
        with self._operation_lock:
            self._ensure_open()
            self._state = "clearing"
            try:
                if self._backend is not None:
                    # The clear command is ordered after all already-enqueued
                    # writes on the single SQLite writer connection.
                    self._backend.clear()
                with self._lock:
                    self._tiles.clear()
                    self._pending.clear()
                    self._visualization_dirty.clear()
                    self._min_ix = self._max_ix = None
                    self._min_iy = self._max_iy = None
                    self._revision += 1
            finally:
                self._state = "open"

    def close(self) -> None:
        with self._operation_lock:
            if self._state == "closed":
                return
            self._ensure_open()
            self._state = "closing"
            backend = self._backend
            failure: Optional[BaseException] = None
            if backend is not None:
                try:
                    self._flush(wait=True)
                except BaseException as exc:
                    failure = exc
                try:
                    backend.close()
                except BaseException as exc:
                    if failure is None:
                        failure = exc
            self._backend = None
            self._state = "closed"
            if failure is not None:
                raise failure

    def cell_center_xy(self, ix: np.ndarray, iy: np.ndarray):
        """Convert integer global cell indices into map-frame centers."""

        with self._operation_lock:
            self._ensure_open()
            return (
                (np.asarray(ix, dtype=np.float64) + self.phase_x)
                * self.resolution,
                (np.asarray(iy, dtype=np.float64) + self.phase_y)
                * self.resolution,
            )

    def _get_tile_locked(self, key: TileKey, create: bool) -> Optional[_TileEntry]:
        entry = self._tiles.get(key)
        if entry is not None:
            self._tiles.move_to_end(key)
            return entry

        pending = self._pending.get(key)
        if pending is not None:
            revision, data = pending
            entry = _TileEntry(data.copy(), revision, -1)
        else:
            loaded = self._backend.load_tile(key, self._tile_shape) if self._backend else None
            if loaded is not None:
                revision, data = loaded
                entry = _TileEntry(data, revision, revision)
            elif create:
                data = np.full(self._tile_shape, np.nan, dtype=np.float32)
                data[self._OBSERVATION_COUNT].fill(0.0)
                data[self._LAST_UPDATE_REVISION].fill(0.0)
                entry = _TileEntry(data, 0, 0)
            else:
                return None
        self._tiles[key] = entry
        self._tiles.move_to_end(key)
        return entry

    def _evict_if_needed_locked(self) -> None:
        if self._backend is None:
            if len(self._tiles) > self.max_tiles_in_memory:
                raise RuntimeError(
                    "Global tile cache reached its in-memory limit without a "
                    "storage_path; configure global_map_storage_path for an "
                    "unbounded route."
                )
            return
        while len(self._tiles) > self.max_tiles_in_memory:
            key, entry = self._tiles.popitem(last=False)
            if entry.revision > entry.persisted_revision:
                self._enqueue_tile_locked(key, entry)

    def _enqueue_tile_locked(self, key: TileKey, entry: _TileEntry) -> None:
        pending = self._pending.get(key)
        if pending is not None and pending[0] >= entry.revision:
            return
        snapshot = entry.data.copy()
        self._backend.enqueue_tile(key, entry.revision, snapshot)
        # The backend owns the immutable snapshot reference; keep the same
        # array here so an evicted tile can be reloaded before its commit.
        self._pending[key] = (entry.revision, snapshot)

    def _on_tile_persisted(self, key: TileKey, revision: int) -> None:
        with self._lock:
            pending = self._pending.get(key)
            if pending is not None and pending[0] == revision:
                del self._pending[key]
            entry = self._tiles.get(key)
            if entry is not None:
                entry.persisted_revision = max(
                    entry.persisted_revision,
                    min(int(revision), entry.revision),
                )

    def _mark_visualization_dirty_locked(self, key: TileKey) -> None:
        self._visualization_dirty.pop(key, None)
        self._visualization_dirty[key] = None

    def _extend_bounds_locked(
        self, min_ix: int, max_ix: int, min_iy: int, max_iy: int
    ) -> None:
        self._min_ix = min_ix if self._min_ix is None else min(self._min_ix, min_ix)
        self._max_ix = max_ix if self._max_ix is None else max(self._max_ix, max_ix)
        self._min_iy = min_iy if self._min_iy is None else min(self._min_iy, min_iy)
        self._max_iy = max_iy if self._max_iy is None else max(self._max_iy, max_iy)

    def _state_metadata_locked(self) -> Dict[str, object]:
        return {
            "schema_version": _SqliteTileBackend._SCHEMA_VERSION,
            "resolution": self.resolution,
            "tile_size_cells": self.tile_size_cells,
            "map_frame": self.map_frame,
            "global_layers": [
                "elevation",
                "variance",
                "observation_count",
                "last_update_revision",
            ],
            "variance_floor": self.variance_floor,
            "variance_ceiling": self.variance_ceiling,
            "outlier_sigma": self.outlier_sigma,
            "max_height_jump": self.max_height_jump,
            "phase_x": self._phase_x,
            "phase_y": self._phase_y,
            "min_ix": self._min_ix,
            "max_ix": self._max_ix,
            "min_iy": self._min_iy,
            "max_iy": self._max_iy,
            "revision": self._revision,
        }

    def _load_state_metadata(self, metadata: Dict[str, object]) -> None:
        if not metadata:
            return
        expected_fusion = {
            "variance_floor": self.variance_floor,
            "variance_ceiling": self.variance_ceiling,
            "outlier_sigma": self.outlier_sigma,
            "max_height_jump": self.max_height_jump,
        }
        for name, expected in expected_fusion.items():
            actual = metadata.get(name)
            if actual is None or not math.isclose(
                float(actual), expected, rel_tol=0.0, abs_tol=1.0e-9
            ):
                raise ValueError(
                    f"Global tile database metadata mismatch for '{name}': "
                    f"database={actual!r}, requested={expected!r}."
                )
        phase_x = metadata.get("phase_x")
        phase_y = metadata.get("phase_y")
        for name, phase in (("phase_x", phase_x), ("phase_y", phase_y)):
            if phase is not None and phase not in (0.0, 0.5):
                raise ValueError(
                    f"Invalid persisted global-map {name}: {phase!r}."
                )
        if (phase_x is None) != (phase_y is None):
            raise ValueError("Persisted global-map lattice phases are incomplete.")

        raw_bounds = [
            metadata.get("min_ix"),
            metadata.get("max_ix"),
            metadata.get("min_iy"),
            metadata.get("max_iy"),
        ]
        if any(value is not None for value in raw_bounds):
            if not all(
                isinstance(value, int) and not isinstance(value, bool)
                for value in raw_bounds
            ):
                raise ValueError("Persisted global-map bounds must be integers.")
            if raw_bounds[0] > raw_bounds[1] or raw_bounds[2] > raw_bounds[3]:
                raise ValueError("Persisted global-map bounds are inverted.")
        else:
            raw_bounds = [None, None, None, None]

        revision = metadata.get("revision") or 0
        if not isinstance(revision, int) or isinstance(revision, bool) or revision < 0:
            raise ValueError(
                f"Invalid persisted global-map revision: {revision!r}."
            )
        self._phase_x = phase_x
        self._phase_y = phase_y
        self._min_ix, self._max_ix, self._min_iy, self._max_iy = raw_bounds
        self._revision = revision

    def _raise_backend_error_locked(self) -> None:
        if self._backend is not None:
            self._backend.raise_if_error()

    def _ensure_open(self) -> None:
        if self._state != "open":
            raise RuntimeError(
                f"Global-map tile store is not open (state={self._state!r})."
            )
