from glob import glob

from setuptools import find_packages, setup

package_name = "gello_crx"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scripts", glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sung Jun",
    maintainer_email="you@lilly.com",
    description="GELLO leader-arm teleoperation for the FANUC CRX-10iA/L.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gello_leader = gello_crx.leader_node:main",
            "gello_teleop = gello_crx.teleop_node:main",
            "gello_gripper = gello_crx.gripper_node:main",
            "gello_calibrate = gello_crx.calibrate:main",
        ],
    },
)
