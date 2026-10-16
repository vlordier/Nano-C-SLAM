from pathlib import Path

from setuptools import find_packages, setup

package_name = "nano_c_slam_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [str(path) for path in Path("launch").glob("*.launch.py")],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vlordier",
    maintainer_email="vlordier@users.noreply.github.com",
    description="ROS 2 telemetry bridge for the Crazyswarm2 Gazebo backend.",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": ["sensor_bridge = nano_c_slam_gazebo.sensor_bridge:main"]
    },
)
