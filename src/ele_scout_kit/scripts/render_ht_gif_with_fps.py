#!/usr/bin/env python3
"""Append measured local/HT receive-rate curves to validated pipeline frames."""

from __future__ import annotations

import argparse
import bisect
import csv
from io import BytesIO
import json
import math
from pathlib import Path
import shutil

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def rolling_rates(times_ns: list[int], window: int = 10) -> np.ndarray:
    result = np.full(len(times_ns), np.nan, dtype=np.float64)
    # Do not report a misleading startup rate from only two or three messages.
    # A plotted value always represents exactly ``window`` receive intervals.
    for index in range(window, len(times_ns)):
        left = index - window
        elapsed = (times_ns[index] - times_ns[left]) / 1_000_000_000.0
        if elapsed > 0.0:
            result[index] = (index - left) / elapsed
    return result


def nearest_index(sorted_values: list[int], target: int) -> int:
    position = bisect.bisect_left(sorted_values, target)
    candidates = [index for index in (position - 1, position) if 0 <= index < len(sorted_values)]
    return min(candidates, key=lambda index: abs(sorted_values[index] - target))


def chart_image(
    time_by_stream: dict[str, np.ndarray],
    rate_by_stream: dict[str, np.ndarray],
    current_time: float,
    width_px: int,
) -> Image.Image:
    dpi = 100
    figure, axis = plt.subplots(figsize=(width_px / dpi, 2.75), dpi=dpi)
    colors = {"local": "#1677ff", "ht": "#e8590c"}
    labels = {"local": "local crop receive FPS", "ht": "HT 8-dir receive FPS"}
    all_finite_rates: list[float] = []
    for stream in ("local", "ht"):
        times = time_by_stream[stream]
        rates = rate_by_stream[stream]
        visible = times <= current_time + 1e-9
        axis.plot(
            times[visible],
            rates[visible],
            color=colors[stream],
            linewidth=2.2,
            label=labels[stream],
        )
        finite = rates[np.isfinite(rates)]
        all_finite_rates.extend(finite.tolist())
        visible_indices = np.flatnonzero(visible & np.isfinite(rates))
        if visible_indices.size:
            value = float(rates[visible_indices[-1]])
            axis.scatter([current_time], [value], color=colors[stream], s=35, zorder=4)
            axis.text(
                0.995,
                0.90 if stream == "local" else 0.75,
                f"{labels[stream]}: {value:.1f} Hz",
                transform=axis.transAxes,
                ha="right",
                va="top",
                color=colors[stream],
                fontsize=11,
                fontweight="bold",
            )
    axis.axhline(20.0, color="#2b8a3e", linestyle="--", linewidth=1.6, label="20 Hz target")
    axis.axvline(current_time, color="#495057", linewidth=1.1, alpha=0.8)
    maximum_time = max(float(values[-1]) for values in time_by_stream.values() if values.size)
    maximum_rate = max([20.0, *all_finite_rates])
    axis.set_xlim(0.0, max(1.0, maximum_time))
    axis.set_ylim(0.0, math.ceil((maximum_rate + 1.0) / 5.0) * 5.0)
    axis.set_xlabel("measured runtime [s]")
    axis.set_ylabel("receive rate [Hz]")
    axis.set_title("Measured ROS receive FPS over time — trailing 10-message window", fontsize=13)
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper left", ncol=3, fontsize=9)
    figure.tight_layout(pad=0.8)
    buffer = BytesIO()
    figure.savefig(buffer, format="png", dpi=dpi, facecolor="white")
    plt.close(figure)
    buffer.seek(0)
    with Image.open(buffer) as image:
        return image.convert("RGB")


def encode_gif(frame_paths: list[Path], output: Path, duration_ms: int) -> None:
    resampling = getattr(Image, "Resampling", Image)
    quantize = getattr(Image, "Quantize", Image)
    palette_frames: list[Image.Image] = []
    for path in frame_paths:
        with Image.open(path) as image:
            frame = image.convert("RGB")
            frame.thumbnail((1400, 980), resampling.LANCZOS)
            palette_frames.append(frame.quantize(colors=192, method=quantize.MEDIANCUT))
    palette_frames[0].save(
        output,
        save_all=True,
        append_images=palette_frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=False,
        disposal=2,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_recording", type=Path)
    parser.add_argument("fps_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--gif-frame-ms", type=int, default=120)
    arguments = parser.parse_args()

    source = arguments.source_recording.resolve()
    output = arguments.output_dir.resolve()
    if output.exists() and any(output.iterdir()):
        raise SystemExit(f"refusing to overwrite non-empty output: {output}")
    frames_dir = output / "gif_frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    events: dict[str, list[dict[str, int]]] = {"local": [], "ht": []}
    with arguments.fps_csv.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            stream = row["stream"]
            if stream in events:
                events[stream].append(
                    {
                        "receive_ns": int(row["receive_monotonic_ns"]),
                        "stamp_ns": int(row["stamp_ns"]),
                    }
                )
    if any(len(values) < 2 for values in events.values()):
        raise SystemExit("FPS CSV does not contain enough local and HT events")
    for values in events.values():
        values.sort(key=lambda event: event["receive_ns"])
    origin_ns = min(values[0]["receive_ns"] for values in events.values())
    time_by_stream = {
        stream: np.asarray(
            [(event["receive_ns"] - origin_ns) / 1_000_000_000.0 for event in values]
        )
        for stream, values in events.items()
    }
    rate_by_stream = {
        stream: rolling_rates([event["receive_ns"] for event in values])
        for stream, values in events.items()
    }
    ht_stamps = [event["stamp_ns"] for event in events["ht"]]
    stamp_order = sorted(range(len(ht_stamps)), key=ht_stamps.__getitem__)
    sorted_stamps = [ht_stamps[index] for index in stamp_order]

    manifest_rows: list[dict[str, str]] = []
    with (source / "manifest.csv").open(newline="", encoding="utf-8") as handle:
        source_rows = list(csv.DictReader(handle))
    output_frames: list[Path] = []
    for row in source_rows:
        source_frame = source / row["png_file"]
        target_stamp = int(row["stamp_ns"])
        sorted_match = nearest_index(sorted_stamps, target_stamp)
        ht_index = stamp_order[sorted_match]
        current_time = float(time_by_stream["ht"][ht_index])
        with Image.open(source_frame) as image:
            base = image.convert("RGB")
        chart = chart_image(time_by_stream, rate_by_stream, current_time, base.width)
        canvas = Image.new("RGB", (base.width, base.height + chart.height), "white")
        canvas.paste(base, (0, 0))
        canvas.paste(chart, (0, base.height))
        target = frames_dir / source_frame.name
        canvas.save(target, format="PNG", optimize=True)
        output_frames.append(target)

        local_visible = np.flatnonzero(
            (time_by_stream["local"] <= current_time) & np.isfinite(rate_by_stream["local"])
        )
        local_rate = (
            float(rate_by_stream["local"][local_visible[-1]])
            if local_visible.size
            else float("nan")
        )
        ht_rate = float(rate_by_stream["ht"][ht_index])
        manifest_rows.append(
            {
                "gif_frame_index": row["gif_frame_index"],
                "stamp_ns": row["stamp_ns"],
                "measured_runtime_s": f"{current_time:.9f}",
                "local_receive_fps_10_message": f"{local_rate:.6f}",
                "ht_receive_fps_10_message": f"{ht_rate:.6f}",
                "png_file": str(target.relative_to(output)),
            }
        )

    gif_path = output / "global_to_local_to_ht_with_fps.gif"
    encode_gif(output_frames, gif_path, arguments.gif_frame_ms)
    with (output / "manifest_with_fps.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(manifest_rows[0]))
        writer.writeheader()
        writer.writerows(manifest_rows)
    shutil.copy2(arguments.fps_csv, output / "measured_topic_events.csv")
    summary = {
        "status": "passed",
        "gif_file": str(gif_path),
        "gif_frames": len(output_frames),
        "gif_frame_duration_ms": arguments.gif_frame_ms,
        "source_recording": str(source),
        "fps_event_source": str(arguments.fps_csv.resolve()),
        "fps_definition": "ROS receive rate over the trailing 10 messages, measured with time.monotonic_ns",
        "local_event_count": len(events["local"]),
        "ht_event_count": len(events["ht"]),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
