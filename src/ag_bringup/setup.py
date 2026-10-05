import os
from glob import glob
from setuptools import setup

package_name = "ag_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kaixuan Xing",
    maintainer_email="you@example.com",
    description="Top-level launch files for the active-view grasping project.",
    license="MIT",
)

