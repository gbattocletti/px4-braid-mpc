"""
Controller launch script. Spawns one PX4-MPC node per robot + the high-level controller.
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # Load sim params
    pkg_share = Path(get_package_share_directory("px4_braid_mpc"))
    with open(pkg_share / "config" / "sim_params.yaml", "r", encoding="utf-8") as f:
        params: dict = yaml.safe_load(f)

    # Extract relevant variables
    namespaces: list[str] = params["namespaces"]
    mpc_mode: str = params["px4_mpc"]["mode"]
    mpc_setpoint_from_rviz: bool = params["px4_mpc"]["setpoint_from_rviz"]

    # Parse params
    if mpc_mode != "wrench":
        raise NotImplementedError(
            f"Invalid mode for PX4-MPC: {mpc_mode}. ",
            "Modes other than 'wrench' are not implemented yet.",
        )
    if mpc_setpoint_from_rviz is True:
        raise NotImplementedError(
            "RViz setpoint input is not implemented yet. ",
            "Set 'setpoint_from_rviz' to false in sim_params.yaml.",
        )

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
                    {"mode": mpc_mode},
                    {"setpoint_from_rviz": mpc_setpoint_from_rviz},
                ],
            ),
        )

    # Append launch action for high-level controller (single node, manages all robots)
    actions.append(
        Node(
            package="px4_braid_mpc",
            executable="high_level_controller",
            name="high_level_controller",
            output="screen",
            emulate_tty=True,
        )
    )

    return LaunchDescription(actions)
