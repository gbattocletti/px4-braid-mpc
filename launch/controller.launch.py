"""
Controller launch script. Spawns one PX4-MPC node per robot + the high-level controller.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:

    # Load sim params
    pkg_share = Path(get_package_share_directory("px4_braid_mpc"))
    with open(pkg_share / "config" / "sim_params.yaml", "r", encoding="utf-8") as f:
        params: dict = yaml.safe_load(f)

    # Extract relevant variables
    namespaces: list[str] = params["namespaces"]

    # Initialize actions list
    actions: list[Node] = []

    # Create list of launch actions (one per robot to spawn the PX4-MPC node)
    for ns in namespaces:
        actions.append(
            Node(
                package="px4_mpc",
                namespace=ns,
                executable="mpc_spacecraft",
                name="mpc_spacecraft",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"mode": "wrench"},
                    {"setpoint_from_rviz": False},
                    {"target_mode": "trajectory"},
                ],
            ),
        )

    # Append launch action for high-level controller (single node, manages all robots)
    # actions.append(
    #     Node(
    #         package="px4_braid_mpc",
    #         executable="high_level_controller",
    #         name="high_level_controller",
    #         output="screen",
    #         emulate_tty=True,
    #     )
    # )

    return LaunchDescription(actions)
