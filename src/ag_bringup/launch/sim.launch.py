"""Bring up Gazebo, the robot, controllers and the perception/execution stack.

Usage:
  ros2 launch ag_bringup sim.launch.py                        # realtime GUI
  ros2 launch ag_bringup sim.launch.py headless:=true fast:=true   # training
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ag_gazebo = get_package_share_directory("ag_gazebo")
    pkg_ag_description = get_package_share_directory("ag_description")
    pkg_ag_perception = get_package_share_directory("ag_perception")
    pkg_ag_execution = get_package_share_directory("ag_execution")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    world_realtime = PathJoinSubstitution([pkg_ag_gazebo, "worlds", "tabletop.world"])
    world_fast = PathJoinSubstitution([pkg_ag_gazebo, "worlds", "tabletop_fast.world"])

    urdf_file = os.path.join(pkg_ag_description, "urdf", "ag_arm.urdf")
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    controller_config = os.path.join(
        pkg_ag_description, "config", "ag_arm_controllers.yaml"
    )
    perception_config = os.path.join(
        pkg_ag_perception, "config", "perception.yaml"
    )
    execution_config = os.path.join(
        pkg_ag_execution, "config", "execution.yaml"
    )

    gazebo_source = os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")

    gazebo_realtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_source),
        launch_arguments={
            "world": world_realtime,
            "gui": LaunchConfiguration("gui"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("fast").__eq__("false")),
    )
    gazebo_fast = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_source),
        launch_arguments={"world": world_fast, "gui": "false"}.items(),
        condition=IfCondition(LaunchConfiguration("fast")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("fast", default_value="false"),
            gazebo_realtime,
            gazebo_fast,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": True}
                ],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "robot_description",
                    "-entity", "ag_arm",
                    "-x", "0.0", "-y", "0.0", "-z", "0.78",
                    "-timeout", "30.0",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    {"robot_description": robot_description}, controller_config
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller"],
                output="screen",
            ),
            Node(
                package="ag_gazebo",
                executable="world_manager",
                output="screen",
            ),
            Node(
                package="ag_perception",
                executable="perception_node",
                parameters=[perception_config],
                output="screen",
            ),
            Node(
                package="ag_execution",
                executable="execution_node",
                parameters=[execution_config, {"urdf_file": urdf_file}],
                output="screen",
            ),
        ]
    )

