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
        launch_arguments={"gui": "true", "fast": "false"}.items(),
    )
    demo = Node(
        package="ag_agent",
        executable="demo",
        name="ag_demo",
        arguments=["--checkpoint", checkpoint, "--objects", "5"],
        output="screen",
    )
    return LaunchDescription(
        [DeclareLaunchArgument("checkpoint", default_value=""), sim, demo]
    )

