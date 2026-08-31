from glob import glob
import os
from pathlib import Path

from setuptools import find_packages, setup


package_name = "ht_traversability_ros2"
# The standalone KIT vendors the validated HT checkpoint with this package.
model_file = os.path.join("models", "student_best_epoch_weights.pth")
model_source = (Path(__file__).resolve().parent / model_file).resolve()
if not model_source.is_file():
    raise FileNotFoundError(f"HT student checkpoint is missing: {model_source}")


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "models"), [model_file]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=False,
    maintainer="HT Traversability Team",
    maintainer_email="maintainer@example.com",
    description="ROS 2 adapter for HT eight-direction traversability inference",
    license="Proprietary research code",
    entry_points={
        "console_scripts": [
            "ht_traversability_node = ht_traversability_ros2.node:main",
            "ht_preflight = ht_traversability_ros2.preflight:main",
        ],
    },
)
