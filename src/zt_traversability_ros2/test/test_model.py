from pathlib import Path

import pytest


torch = pytest.importorskip("torch")

from zt_traversability_ros2.model import load_student_model


def test_packaged_checkpoint_loads_strictly_and_has_expected_shape():
    weight = Path(__file__).parents[1] / "models" / "student_8dir.pth"
    model = load_student_model(str(weight), torch.device("cpu"))
    with torch.no_grad():
        output = model(torch.zeros((1, 1, 45, 45), dtype=torch.float32))
    assert tuple(output.shape) == (1, 16, 1, 1)
