import os
from glob import glob
from setuptools import setup

package_name = "ag_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kaixuan Xing",
    maintainer_email="you@example.com",
    description="DDQN agent with active viewpoint selection for tabletop decluttering.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "train = ag_agent.train:main",
            "evaluate = ag_agent.evaluate:main",
            "demo = ag_agent.demo:main",
        ],
    },
)

