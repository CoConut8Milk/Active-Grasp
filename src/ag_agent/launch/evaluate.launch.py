import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ag_bringup = get_package_share_directory("ag_bringup")
    checkpoint = LaunchConfiguration("checkpoint")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ag_bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={"gui": "false", "fast": "true"}.items(),
    )
    evaluate = Node(
        package="ag_agent",
        executable="evaluate",
        name="ag_evaluate",
        arguments=[
            "--checkpoint", checkpoint,
            "--episodes", "10",
            "--objects", "2", "4", "6", "8",
            "--out", "results/evaluation.json",
        ],
        output="screen",
    )
    return LaunchDescription(
        [DeclareLaunchArgument("checkpoint"), sim, evaluate]
    )

