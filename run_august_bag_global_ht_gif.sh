#!/usr/bin/env bash
set -Eeo pipefail

if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "usage: $0 BAG_DIR OUTPUT_DIR [ROS_DOMAIN_ID]" >&2
  exit 2
fi

kit_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
bag_dir="$(realpath "$1")"
output_dir="$2"
domain_id="${3:-191}"
wait_after="${OFFLINE_WAIT_AFTER:-1.0}"
auto_steps="${OFFLINE_AUTO_STEPS:-999999}"
frame_stride="${OFFLINE_FRAME_STRIDE:-1}"

if [[ ! -f "${bag_dir}/metadata.yaml" ]]; then
  echo "not a ROS 2 bag: ${bag_dir}" >&2
  exit 2
fi
if [[ ! -r "${kit_root}/install/local_setup.bash" ]]; then
  echo "kit is not built; run: bash ${kit_root}/build_workspace.sh" >&2
  exit 2
fi
if [[ -e "${output_dir}" ]] && [[ -n "$(find "${output_dir}" -mindepth 1 -print -quit 2>/dev/null)" ]]; then
  echo "refusing to overwrite non-empty output: ${output_dir}" >&2
  exit 2
fi

mkdir -p "${output_dir}/logs" "${output_dir}/indexes" "${output_dir}/recording"
output_dir="$(realpath "${output_dir}")"
database_path="${output_dir}/global_tiles.sqlite3"

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
unset LD_LIBRARY_PATH PKG_CONFIG_PATH CPATH CPLUS_INCLUDE_PATH LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "${kit_root}/install/local_setup.bash"
export ROS_DOMAIN_ID="${domain_id}"
export PYTHONUNBUFFERED=1
source "${kit_root}/setup_runtime_env.sh"
unset FASTDDS_BUILTIN_TRANSPORTS

pipeline_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e
  if [[ -n "${pipeline_pid}" ]]; then
    # launch and all three ROS nodes live in their own session/process group.
    kill -INT -- "-${pipeline_pid}" 2>/dev/null
    sleep 2
    kill -TERM -- "-${pipeline_pid}" 2>/dev/null
    wait "${pipeline_pid}" 2>/dev/null
  fi
  exit "${status}"
}
trap cleanup EXIT INT TERM

# Isolate launch from this runner.  launch's shutdown handler signals its own
# process group; without setsid that signal also terminates this parent script
# before it can validate and summarize the generated GIF/database.
setsid ros2 launch ele_scout_kit offline_global_ht_pipeline.launch.py \
  output_dir:="${output_dir}/recording" \
  database_path:="${database_path}" \
  frame_stride:="${frame_stride}" \
  gif_frame_ms:=120 \
  >"${output_dir}/logs/pipeline.log" 2>&1 &
pipeline_pid=$!

ready=0
for _attempt in $(seq 1 240); do
  if ! kill -0 "${pipeline_pid}" 2>/dev/null; then
    echo "pipeline exited during startup" >&2
    tail -120 "${output_dir}/logs/pipeline.log" >&2
    exit 3
  fi
  if rg -q "Unbounded tiled global elevation map enabled" "${output_dir}/logs/pipeline.log" \
    && rg -q "Ready: input=/elevation_mapping_node/elevation_map_global_local" "${output_dir}/logs/pipeline.log" \
    && rg -q "Recording detailed HT diagnostic frames" "${output_dir}/logs/pipeline.log"; then
    ready=1
    break
  fi
  sleep 0.5
done
if [[ "${ready}" -ne 1 ]]; then
  echo "timed out waiting for mapping, HT, and GIF recorder" >&2
  tail -160 "${output_dir}/logs/pipeline.log" >&2
  exit 3
fi

python3 "${kit_root}/install/ele_scout_kit/lib/ele_scout_kit/step_bag_player.py" \
  "${bag_dir}" \
  --mode stable \
  --index-dir "${output_dir}/indexes" \
  --auto-steps "${auto_steps}" \
  --ack-topic /elevation_mapping_node/processed_pointcloud_count \
  --receipt-topic /elevation_mapping_node/received_pointcloud_count \
  --ack-timeout 120.0 \
  --receipt-timeout 2.0 \
  --direct-cloud-publish \
  --wait-after "${wait_after}" \
  >"${output_dir}/logs/player.log" 2>&1

# Let the recorder consume the last queued HT frame before requesting GIF finalization.
for _attempt in $(seq 1 60); do
  received="$(rg -o 'Saved detailed PNG frames: [0-9]+' "${output_dir}/logs/pipeline.log" | tail -1 | awk '{print $5}')"
  if [[ -n "${received}" ]]; then
    break
  fi
  sleep 0.2
done
sleep 3

# Signal the isolated process group so the mapper flushes SQLite and the
# recorder reaches its finally block to encode the GIF.  Killing only the
# ros2-launch PID can orphan all three child nodes.
kill -INT -- "-${pipeline_pid}"
for _attempt in $(seq 1 240); do
  if [[ -f "${output_dir}/recording/summary.json" ]]; then
    break
  fi
  sleep 0.5
done
if [[ ! -f "${output_dir}/recording/summary.json" ]]; then
  kill -TERM -- "-${pipeline_pid}" 2>/dev/null || true
  echo "timed out waiting for HT GIF finalization" >&2
  exit 4
fi
wait "${pipeline_pid}" || launch_status=$?
pipeline_pid=""

OUTPUT_DIR="${output_dir}" BAG_DIR="${bag_dir}" DATABASE_PATH="${database_path}" python3 - <<'PY'
import json
import os
from pathlib import Path
import sqlite3

output = Path(os.environ["OUTPUT_DIR"])
bag = Path(os.environ["BAG_DIR"])
database = Path(os.environ["DATABASE_PATH"])
recording = json.loads((output / "recording" / "summary.json").read_text())
connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
try:
    metadata = {
        key: json.loads(value)
        for key, value in connection.execute("SELECT key, value FROM metadata")
    }
    tile_count = int(connection.execute("SELECT COUNT(*) FROM tiles WHERE level=0").fetchone()[0])
    integrity = connection.execute("PRAGMA integrity_check").fetchone()[0]
finally:
    connection.close()

summary = {
    "status": "passed" if recording["status"] == "passed" and integrity == "ok" else "failed",
    "source_bag": str(bag),
    "output_dir": str(output),
    "global_database": str(database),
    "global_database_revision": int(metadata["revision"]),
    "global_tile_count": tile_count,
    "sqlite_integrity": integrity,
    "gif": recording,
}
(output / "summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
)
print(json.dumps(summary, indent=2, ensure_ascii=False))
if summary["status"] != "passed":
    raise SystemExit("offline global-to-HT pipeline validation failed")
PY

trap - EXIT INT TERM
echo "done: ${output_dir}"
