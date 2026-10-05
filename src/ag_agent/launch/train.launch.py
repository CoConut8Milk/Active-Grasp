import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ag_bringup = get_package_share_directory("ag_bringup")
    pkg_ag_agent = get_package_share_directory("ag_agent")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ag_bringup, "launch", "sim.launch.py")
        ),
        launch_arguments={"gui": "false", "fast": "true"}.items(),
    )
    train = Node(
        package="ag_agent",
        executable="train",
        name="ag_train",
        parameters=[os.path.join(pkg_ag_agent, "config", "agent.yaml")],
        output="screen",
    )
    return LaunchDescription([sim, train])

