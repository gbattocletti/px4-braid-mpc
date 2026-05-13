"""
Launcher for the rviz visualization stack.

Spawns:
- a ros2 node that publishes the world boundaries
- 3 px4-offboard visualizer nodes (one per robot) that visualize the setpoints and
  trajectories of each robot
- an rviz2 instance with a pre-configured view (see config/config.rviz)

Usage:
    ros2 launch px4_braid_mpc rviz.launch.py
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    pkg_share = Path(get_package_share_directory("px4_braid_mpc"))
    rviz_config = pkg_share / "config" / "config.rviz"
    with open(pkg_share / "config" / "sim_params.yaml", "r", encoding="utf-8") as f:
        params: dict = yaml.safe_load(f)

    # Extract relevant variables
    namespaces: list[str] = params["namespaces"]

    # Initialize actions list
    actions: list[Node] = []

    # px4-offboard visualizer nodes (one per robot)
    for ns in namespaces:
        actions.append(
            Node(
                package="px4_offboard",
                namespace=ns,
                executable="visualizer",
                name="visualizer",
                output="screen",
                emulate_tty=True,
            )
        )

    # Boundary publisher
    actions.append(
        Node(
            package="px4_world_publisher",
            namespace="",
            executable="boundary_publisher",
            name="px4_world_publisher",
            output="screen",
            emulate_tty=True,
        )
    )

    # RViz
    actions.append(
        Node(
            package="rviz2",
            namespace="",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", str(rviz_config)],
            output="screen",
        )
    )

    return LaunchDescription(actions)
