"""Legacy ZT student network matching the supplied 16-channel checkpoint.

This is a cleaned inference-only copy of
``ht_traversability_new_F_student_grad_res_c_16_reduce_predict`` from the
provided ZT archive.  The network is fully convolutional: a HxW elevation
grid produces 16x(H-44)x(W-44) logits.
"""

from __future__ import annotations

from typing import Mapping

import torch
from torch import Tensor, nn


class BasicBlock(nn.Module):
    def __init__(self, channels: int) -> None:
        super().__init__()
        self.conv1 = nn.Conv2d(channels, channels, kernel_size=3, bias=False)
        self.bn1 = nn.BatchNorm2d(channels)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(channels, channels, kernel_size=3, bias=False)
        self.bn2 = nn.BatchNorm2d(channels)

    def forward(self, x: Tensor) -> Tensor:
        identity = x
        out = self.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out = out + identity[:, :, 2:-2, 2:-2]
        return self.relu(out)


class HeightGradientActivation(nn.Module):
    """The original activation is a constant scale, not a trainable layer."""

    def __init__(self, slope: float = 1.0 / 0.06412482246885533) -> None:
        super().__init__()
        self.slope = float(slope)

    def forward(self, x: Tensor) -> Tensor:
        return x * self.slope


class ZTStudent8Direction(nn.Module):
    """Five-stage legacy ZT student producing 8 directions x 2 classes."""

    receptive_field = 45
    direction_count = 8
    class_count = 2

    def __init__(self) -> None:
        super().__init__()
        self.conv_grad = nn.Conv2d(1, 8, kernel_size=3, bias=False)
        self.ht_grad_activate = HeightGradientActivation()

        self.res_conv1 = nn.Conv2d(8, 16, kernel_size=3, bias=False)
        self.res_bn1 = nn.BatchNorm2d(16)
        self.res_block1 = BasicBlock(16)

        self.res_conv2 = nn.Conv2d(16, 24, kernel_size=3, bias=False)
        self.res_bn2 = nn.BatchNorm2d(24)
        self.res_block2 = BasicBlock(24)

        self.res_conv3 = nn.Conv2d(24, 32, kernel_size=3, bias=False)
        self.res_bn3 = nn.BatchNorm2d(32)
        self.res_block3 = BasicBlock(32)

        self.res_conv4 = nn.Conv2d(32, 40, kernel_size=3, bias=False)
        self.res_bn4 = nn.BatchNorm2d(40)
        self.res_block4 = BasicBlock(40)

        self.res_conv5 = nn.Conv2d(40, 48, kernel_size=3, bias=False)
        self.res_bn5 = nn.BatchNorm2d(48)
        self.res_block5 = BasicBlock(48)

        self.hdc_conv1 = nn.Conv2d(48, 56, kernel_size=3, dilation=1, bias=False)
        self.hdc_bn1 = nn.BatchNorm2d(56)
        self.hdc_conv2 = nn.Conv2d(56, 64, kernel_size=3, dilation=2, bias=False)
        self.hdc_bn2 = nn.BatchNorm2d(64)
        self.hdc_conv3 = nn.Conv2d(64, 72, kernel_size=3, dilation=3, bias=False)
        self.hdc_bn3 = nn.BatchNorm2d(72)

        self.conv_dim = nn.Conv2d(72, 16, kernel_size=1, bias=True)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x: Tensor) -> Tensor:
        x = self.ht_grad_activate(self.conv_grad(x))

        x = self.relu(self.res_bn1(self.res_conv1(x)))
        x = self.res_block1(x)
        x = self.relu(self.res_bn2(self.res_conv2(x)))
        x = self.res_block2(x)
        x = self.relu(self.res_bn3(self.res_conv3(x)))
        x = self.res_block3(x)
        x = self.relu(self.res_bn4(self.res_conv4(x)))
        x = self.res_block4(x)
        x = self.relu(self.res_bn5(self.res_conv5(x)))
        x = self.res_block5(x)

        x = self.relu(self.hdc_bn1(self.hdc_conv1(x)))
        x = self.relu(self.hdc_bn2(self.hdc_conv2(x)))
        x = self.relu(self.hdc_bn3(self.hdc_conv3(x)))
        return self.conv_dim(x)


def normalize_checkpoint(checkpoint: object) -> Mapping[str, Tensor]:
    """Return a plain state_dict and tolerate common training wrappers."""

    if not isinstance(checkpoint, Mapping):
        raise TypeError(f"Checkpoint must be a mapping, got {type(checkpoint).__name__}")

    for key in ("state_dict", "model_state_dict", "model"):
        wrapped = checkpoint.get(key)
        if isinstance(wrapped, Mapping):
            checkpoint = wrapped
            break

    state = {}
    for key, value in checkpoint.items():
        if not isinstance(key, str) or not isinstance(value, Tensor):
            continue
        state[key.removeprefix("module.")] = value
    if not state:
        raise ValueError("Checkpoint does not contain tensor parameters")
    return state


def load_student_model(weight_path: str, device: torch.device) -> ZTStudent8Direction:
    """Load the supplied checkpoint strictly, then switch to inference mode."""

    try:
        checkpoint = torch.load(weight_path, map_location="cpu", weights_only=True)
    except TypeError:  # Older Jetson PyTorch builds do not expose weights_only.
        checkpoint = torch.load(weight_path, map_location="cpu")

    model = ZTStudent8Direction()
    model.load_state_dict(normalize_checkpoint(checkpoint), strict=True)
    model.to(device)
    model.eval()
    return model
