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

import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:

    # Get path to package share and read sim params
    pkg_share = Path(get_package_share_directory("px4_braid_mpc"))
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

    # Patch rviz config to use the correct topic names
    rviz_config = generate_rviz_config(pkg_share, namespaces)

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


def generate_rviz_config(pkg_share: Path, namespaces: list[str]) -> str:
    """
    Assemble the final rviz config from `config.rviz` and `agent.rviz`,
    using namespaces from `sim_params.yaml`.

    Args:
        pkg_share (Path): path to the package share directory.
        namespaces (list[str]): list of namespaces to visualize.

    Returns:
        str: path to a temp config file.
    """
    # Read base config
    with open(pkg_share / "config" / "config.rviz", "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    # Read per-agent template (for placeholder substitution)
    with open(pkg_share / "config" / "agent.rviz", "r", encoding="utf-8") as f:
        agent_tmpl = f.read()

    # For each namespace, render the template and append the resulting displays to the
    # base config's Displays list.
    displays = config["Visualization Manager"]["Displays"]
    for ns in namespaces:
        rendered = agent_tmpl.replace("__NS__", ns)
        displays.extend(yaml.safe_load(rendered))

    # write to a temp config file and return its path
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".rviz", delete=False, prefix="braid_mpc_"
    ) as tmp:
        yaml.safe_dump(config, tmp, sort_keys=False)
        return tmp.name
