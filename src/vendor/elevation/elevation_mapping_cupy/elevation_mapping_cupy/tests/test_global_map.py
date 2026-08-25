import numpy as np
import pytest
import threading

from elevation_mapping_cupy.global_map import (
    GlobalMapAccumulator,
    downsample_elevation_blocks,
)


def _make_global(**overrides):
    params = {
        "resolution": 1.0,
        "tile_size_cells": 2,
        "max_tiles_in_memory": 8,
        "storage_path": "",
        "resume_from_disk": False,
        "map_frame": "map",
    }
    params.update(overrides)
    result = GlobalMapAccumulator(**params)
    result.initialize_lattice((2, 2))
    return result


def test_latest_valid_wins_and_nan_does_not_erase_history():
    global_map = _make_global()
    first = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    result = global_map.fuse_snapshot((0.0, 0.0), first)

    assert result.written_cells == 4
    assert result.touched_tiles == 4  # The origin crosses four signed tiles.
    assert set(global_map.all_tile_keys()) == {(-1, -1), (0, -1), (-1, 0), (0, 0)}

    second = np.array([[9.0, np.nan], [np.nan, 8.0]], dtype=np.float32)
    global_map.fuse_snapshot((0.0, 0.0), second)
    window = global_map.extract_window((0.0, 0.0), 2.0, 2.0)
    expected = np.array([[9.0, 2.0], [3.0, 8.0]], dtype=np.float32)
    assert np.array_equal(window.elevation_yx, expected)


def test_far_jump_allocates_only_observed_tiles_not_intermediate_rectangle():
    global_map = _make_global(tile_size_cells=4)
    local = np.ones((2, 2), dtype=np.float32)
    global_map.fuse_snapshot((0.0, 0.0), local)
    keys_near_origin = set(global_map.all_tile_keys())
    global_map.fuse_snapshot((100.0, -100.0), local * 7.0)
    keys_after_jump = set(global_map.all_tile_keys())

    assert len(keys_near_origin) == 4
    assert len(keys_after_jump) == 8
    assert (12, 0) not in keys_after_jump


def test_window_xy_axes_and_gridmap_conversion_use_independent_oracle():
    global_map = _make_global(tile_size_cells=4)
    natural = np.array([[11.0, 12.0], [21.0, 22.0]], dtype=np.float32)
    global_map.fuse_snapshot((0.0, 0.0), natural)
    window = global_map.extract_window((0.0, 0.0), 2.0, 2.0)

    assert np.array_equal(window.elevation_yx, natural)
    # GridMap row/column zero is the +X/+Y corner.
    expected_grid_map = np.array([[22.0, 12.0], [21.0, 11.0]], dtype=np.float32)
    assert np.array_equal(window.elevation_grid_map_copy(), expected_grid_map)


@pytest.mark.parametrize("shape", [(3, 3), (2, 3), (3, 2), (5, 4)])
def test_odd_and_mixed_parity_lattices_round_trip(shape):
    global_map = GlobalMapAccumulator(
        resolution=1.0,
        tile_size_cells=3,
        max_tiles_in_memory=16,
    )
    global_map.initialize_lattice(shape)
    source = np.arange(np.prod(shape), dtype=np.float32).reshape(shape) + 1.0
    center = (-6.0, 4.0)

    global_map.fuse_snapshot(center, source)
    restored = global_map.extract_window(center, shape[1], shape[0])

    assert restored.elevation_yx.shape == shape
    assert np.array_equal(restored.elevation_yx, source)


def test_far_negative_tile_boundaries_keep_exact_values():
    global_map = GlobalMapAccumulator(
        resolution=1.0,
        tile_size_cells=3,
        max_tiles_in_memory=16,
    )
    global_map.initialize_lattice((2, 3))
    source = np.array([[1.0, 2.0, 3.0], [11.0, 12.0, 13.0]], dtype=np.float32)

    global_map.fuse_snapshot((-6.0, -3.0), source)

    assert set(global_map.all_tile_keys()) == {(-3, -2), (-2, -2), (-3, -1), (-2, -1)}
    restored = global_map.extract_window((-6.0, -3.0), 3.0, 2.0)
    assert np.array_equal(restored.elevation_yx, source)


def test_float32_center_drift_is_snapped_to_integer_cells():
    global_map = GlobalMapAccumulator(
        resolution=0.04,
        tile_size_cells=128,
        max_tiles_in_memory=16,
    )
    global_map.initialize_lattice((250, 250))
    # Reproduces float32 accumulation after hundreds of 0.04 m shifts.
    drifted_center = np.float32(18.36004)
    start_x, start_y = global_map.validate_snapshot_geometry(
        (drifted_center, np.float32(-18.36004)), (250, 250)
    )
    assert start_x == 459 - 125
    assert start_y == -459 - 125


def test_genuine_fractional_cell_misalignment_is_rejected():
    global_map = _make_global()
    with pytest.raises(ValueError, match="too far"):
        global_map.fuse_snapshot((0.4, 0.0), np.ones((2, 2), dtype=np.float32))


def test_empty_window_read_does_not_allocate_tiles():
    global_map = _make_global(tile_size_cells=4)
    assert np.isnan(
        global_map.extract_window((100.0, -100.0), 8.0, 8.0).elevation_yx
    ).all()
    assert global_map.tile_count() == 0


def test_block_downsampling_preserves_thin_observations_and_block_centers():
    tile = np.full((4, 4), np.nan, dtype=np.float32)
    tile[1, 1] = 2.0  # Not on the old [::2, ::2] sampling lattice.
    tile[2, 2] = 4.0
    tile[3, 3] = 8.0

    x, y, z = downsample_elevation_blocks(tile, stride=2)

    assert np.array_equal(x, np.array([0.5, 2.5], dtype=np.float32))
    assert np.array_equal(y, np.array([0.5, 2.5], dtype=np.float32))
    assert np.allclose(z, np.array([2.0, 6.0], dtype=np.float32))


def test_extract_window_preserves_old_and_new_distant_areas():
    global_map = _make_global(tile_size_cells=4)
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    global_map.fuse_snapshot((6.0, 0.0), np.full((2, 2), 7.0, dtype=np.float32))

    old_window = global_map.extract_window((0.0, 0.0), 2.0, 2.0)
    new_window = global_map.extract_window((6.0, 0.0), 2.0, 2.0)
    assert np.all(old_window.elevation_yx == 1.0)
    assert np.all(new_window.elevation_yx == 7.0)


def test_sqlite_eviction_reload_and_resume(tmp_path):
    db_path = tmp_path / "global_tiles.sqlite3"
    global_map = _make_global(
        tile_size_cells=2,
        max_tiles_in_memory=1,
        storage_path=str(db_path),
    )
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    global_map.fuse_snapshot((10.0, 10.0), np.full((2, 2), 5.0, dtype=np.float32))
    global_map.flush(wait=True)
    assert global_map.cached_tile_count <= 1
    assert global_map.tile_count() == 8
    global_map.close()

    resumed = GlobalMapAccumulator(
        resolution=1.0,
        tile_size_cells=2,
        max_tiles_in_memory=1,
        storage_path=str(db_path),
        resume_from_disk=True,
        map_frame="map",
    )
    resumed.initialize_lattice((2, 2))
    assert np.all(resumed.extract_window((0.0, 0.0), 2.0, 2.0).elevation_yx == 1.0)
    assert np.all(resumed.extract_window((10.0, 10.0), 2.0, 2.0).elevation_yx == 5.0)
    resumed.close()


def test_resume_rejects_geometry_mismatch(tmp_path):
    db_path = tmp_path / "global_tiles.sqlite3"
    global_map = _make_global(storage_path=str(db_path))
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    global_map.close()

    with pytest.raises(ValueError, match="metadata mismatch"):
        GlobalMapAccumulator(
            resolution=0.5,
            tile_size_cells=2,
            max_tiles_in_memory=8,
            storage_path=str(db_path),
            resume_from_disk=True,
            map_frame="map",
        )


def test_resume_rejects_local_lattice_parity_mismatch(tmp_path):
    db_path = tmp_path / "global_tiles.sqlite3"
    global_map = _make_global(storage_path=str(db_path))
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    global_map.close()

    resumed = GlobalMapAccumulator(
        resolution=1.0,
        tile_size_cells=2,
        max_tiles_in_memory=8,
        storage_path=str(db_path),
        resume_from_disk=True,
        map_frame="map",
    )
    with pytest.raises(ValueError, match="parity"):
        resumed.initialize_lattice((3, 2))
    resumed.close()


def test_late_visualizer_can_queue_and_replay_all_tiles():
    global_map = _make_global(tile_size_cells=4)
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    global_map.take_visualization_tiles(100)
    global_map.queue_all_tiles_for_visualization()

    replay = global_map.take_visualization_tiles(100)
    assert {key for key, _ in replay} == set(global_map.all_tile_keys())


def test_clear_removes_ram_and_disk_tiles(tmp_path):
    global_map = _make_global(storage_path=str(tmp_path / "tiles.sqlite3"))
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    global_map.flush(wait=True)
    assert global_map.has_observations
    assert global_map.tile_count() == 4

    global_map.clear()

    assert not global_map.has_observations
    assert global_map.tile_count() == 0
    assert np.isnan(
        global_map.extract_window((0.0, 0.0), 2.0, 2.0).elevation_yx
    ).all()
    global_map.close()


def test_clear_serializes_a_concurrent_fusion_after_the_delete(tmp_path, monkeypatch):
    global_map = _make_global(storage_path=str(tmp_path / "tiles.sqlite3"))
    global_map.fuse_snapshot((0.0, 0.0), np.ones((2, 2), dtype=np.float32))
    entered_clear = threading.Event()
    release_clear = threading.Event()
    original_clear = global_map._backend.clear

    def blocking_clear():
        entered_clear.set()
        assert release_clear.wait(timeout=5.0)
        original_clear()

    monkeypatch.setattr(global_map._backend, "clear", blocking_clear)
    clear_thread = threading.Thread(target=global_map.clear)
    clear_thread.start()
    assert entered_clear.wait(timeout=5.0)

    fuse_done = threading.Event()

    def fuse_after_clear():
        global_map.fuse_snapshot(
            (10.0, 10.0), np.full((2, 2), 7.0, dtype=np.float32)
        )
        fuse_done.set()

    fuse_thread = threading.Thread(target=fuse_after_clear)
    fuse_thread.start()
    assert not fuse_done.wait(timeout=0.1)
    release_clear.set()
    clear_thread.join(timeout=5.0)
    fuse_thread.join(timeout=5.0)

    assert fuse_done.is_set()
    assert np.all(
        global_map.extract_window((10.0, 10.0), 2.0, 2.0).elevation_yx == 7.0
    )
    global_map.close()


def test_closed_store_rejects_further_writes():
    global_map = _make_global()
    global_map.close()
    with pytest.raises(RuntimeError, match="not open"):
        global_map.fuse_snapshot(
            (0.0, 0.0), np.ones((2, 2), dtype=np.float32)
        )


def test_memory_only_store_fails_at_cache_limit_instead_of_growing_forever():
    global_map = GlobalMapAccumulator(
        resolution=1.0,
        tile_size_cells=2,
        max_tiles_in_memory=1,
    )
    global_map.initialize_lattice((2, 2))
    with pytest.raises(RuntimeError, match="storage_path"):
        global_map.fuse_snapshot(
            (0.0, 0.0), np.ones((2, 2), dtype=np.float32)
        )
