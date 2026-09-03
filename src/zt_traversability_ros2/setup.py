from glob import glob
import os

from setuptools import find_packages, setup


package_name = "zt_traversability_ros2"


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
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="LENOVO",
    maintainer_email="maintainer@example.com",
    description="ROS 2 adapter for ZT eight-direction traversability inference",
    license="Proprietary research code",
    entry_points={
        "console_scripts": [
            "zt_traversability_node = zt_traversability_ros2.node:main",
            "zt_preflight = zt_traversability_ros2.preflight:main",
        ],
    },
)
