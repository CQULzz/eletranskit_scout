from pathlib import Path

import pytest


torch = pytest.importorskip("torch")

from ht_traversability_ros2.model import (
    GRADIENT_KERNEL,
    HTStudent8Direction,
    load_student_model,
)


def checkpoint_path() -> Path:
    return Path(__file__).resolve().parents[1] / "models" / "student_best_epoch_weights.pth"


def test_packaged_checkpoint_loads_strictly_and_has_expected_shape():
    model = load_student_model(str(checkpoint_path()), torch.device("cpu"))
    with torch.no_grad():
        output = model(torch.zeros((1, 1, 51, 51), dtype=torch.float32))
        probabilities = torch.softmax(output, dim=2)
    assert tuple(output.shape) == (1, 8, 2, 1, 1)
    torch.testing.assert_close(probabilities.sum(dim=2), torch.ones((1, 8, 1, 1)))


def test_dense_output_size_and_fixed_gradient_encoder():
    model = HTStudent8Direction().eval()
    assert not model.conv_grad.weight.requires_grad
    torch.testing.assert_close(model.conv_grad.weight.detach(), GRADIENT_KERNEL)
    with torch.no_grad():
        output = model(torch.zeros((1, 1, 64, 80), dtype=torch.float32))
    assert tuple(output.shape) == (1, 8, 2, 14, 30)
