"""Check HT dependencies, checkpoint integrity, CUDA, and output shape."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


EXPECTED_SHA256 = "0a79b865d012aa633fbb9913fc4031908dfe188ec9e161ef5a3fbb34a3f8c4d4"


def _default_model() -> Path:
    return (
        Path(get_package_share_directory("ht_traversability_ros2"))
        / "models"
        / "student_best_epoch_weights.pth"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Check HT model installation")
    parser.add_argument("--model", type=Path, default=None)
    parser.add_argument("--device", default="cuda", choices=("cuda", "cpu"))
    args = parser.parse_args()

    import numpy as np
    import scipy
    import torch

    from .model import load_student_model

    model_path = (args.model or _default_model()).expanduser().resolve()
    if not model_path.is_file():
        raise SystemExit(f"FAIL: weight not found: {model_path}")
    digest = hashlib.sha256(model_path.read_bytes()).hexdigest()
    if digest != EXPECTED_SHA256:
        raise SystemExit(
            f"FAIL: checkpoint SHA256 mismatch: expected={EXPECTED_SHA256}, actual={digest}"
        )

    if args.device == "cuda" and not torch.cuda.is_available():
        raise SystemExit("FAIL: CUDA requested but torch.cuda.is_available() is false")
    device = torch.device(args.device)
    model = load_student_model(str(model_path), device)
    with torch.no_grad():
        output = model(torch.zeros((1, 1, 51, 51), dtype=torch.float32, device=device))
        probability = torch.softmax(output, dim=2)
    if tuple(output.shape) != (1, 8, 2, 1, 1):
        raise SystemExit(f"FAIL: unexpected output shape {tuple(output.shape)}")
    if not bool(torch.isfinite(probability).all()):
        raise SystemExit("FAIL: model probability contains non-finite values")

    print(f"PASS: numpy={np.__version__}, scipy={scipy.__version__}, torch={torch.__version__}")
    print(f"PASS: device={device}, checkpoint={model_path}")
    print(f"PASS: checkpoint_sha256={digest}")
    print("PASS: input [1,1,51,51] -> logits [1,8,2,1,1]")


if __name__ == "__main__":
    main()
