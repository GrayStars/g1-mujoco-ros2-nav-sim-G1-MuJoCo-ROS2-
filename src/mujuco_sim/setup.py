from glob import glob
import os

from setuptools import find_packages, setup

package_name = "mujuco_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.xml")),
        (os.path.join("share", package_name, "rviz"), glob("*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xkh",
    maintainer_email="xkh@example.com",
    description="MuJoCo to ROS2 bridge for navigation simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "unitree_mujoco_shm_bridge = mujuco_sim.unitree_mujoco_shm_bridge:main",
            "fastlio_tf_bridge = mujuco_sim.fastlio_tf_bridge:main",
        ],
    },
)
