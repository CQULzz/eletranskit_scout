"""On-vehicle dependency, checkpoint, and output-shape self-check."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


EXPECTED_SHA256 = "4fb02eadc146555175faf3920eea39e0104ec292855036f83e455fc4af15fd06"


def _default_model() -> Path:
    return (
        Path(get_package_share_directory("zt_traversability_ros2"))
        / "models"
        / "student_8dir.pth"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Check ZT model installation")
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
        print(f"WARNING: model SHA256 differs from packaged checkpoint: {digest}")

    if args.device == "cuda" and not torch.cuda.is_available():
        raise SystemExit("FAIL: CUDA requested but torch.cuda.is_available() is false")
    device = torch.device(args.device)
    model = load_student_model(str(model_path), device)
    with torch.no_grad():
        output = model(torch.zeros((1, 1, 45, 45), dtype=torch.float32, device=device))
    if tuple(output.shape) != (1, 16, 1, 1):
        raise SystemExit(f"FAIL: unexpected output shape {tuple(output.shape)}")

    print(f"PASS: numpy={np.__version__}, scipy={scipy.__version__}, torch={torch.__version__}")
    print(f"PASS: device={device}, checkpoint={model_path}")
    print("PASS: input [1,1,45,45] -> logits [1,16,1,1]")


if __name__ == "__main__":
    main()
