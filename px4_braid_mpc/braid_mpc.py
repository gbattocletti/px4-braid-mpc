"""
Braid MPC node for PX4-based robots in the ATMOS simulator.
"""

from functools import partial
from pathlib import Path as FilePath

import numpy as np
import yaml
from ament_index_python import get_package_share_directory
from nav_msgs.msg import Path
from px4_msgs.msg import (
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.subscription import Subscription
from trajectory_msgs.msg import MultiDOFJointTrajectory

import px4_braid_mpc.messages as messages
from px4_braid_mpc import interpolation


class BraidMPC(Node):
    """
    ROS2 node that implements the distributed braid MPC algorithm for a team of
    PX4-based robots in the ATMOS simulator. The node subscribes to state measurements
    from the PX4-FMU nodes for each robot, and publishes a reference trajectory for each
    of the robots. The trajectories are then tracked by a PX4-MPC node running for each
    robot, which then sends the control commands to the PX4-FMU nodes for execution.

    NOTE: the braid MPC algorithm is designed to be implemented with a distributed
    scheme. For simplicity, in the current implementation all the 3 braid MPC
    controllers are run within the same node. This allows to reduce the number of topics
    and the amount of inter-node communication. However, we highlight that the
    information used by each MCP controller is still local to a specific robot, and the
    only difference from a truly distributed implementation is a reduced communication
    overhead, which would in any case be minimal due to the small amound of information
    shared between the controllers, and the fact that all the controllers are running on
    the same machine, with the only information published externally being the control
    signals for the PX4 flight controllers.
    """

    def __init__(self):
        super().__init__("braid_mpc")

        # Parameters and namespaces
        self.params: dict
        self.namespaces: list[str]

        # Publishers and listeners
        # TODO: check what topics are actually used and remove the unnecessary ones
        # (also remove the corresponding subscribers below)
        self.reference_traj_pub: dict[str, Publisher] = {}
        self.reference_path_pub: dict[str, Publisher] = {}
        self.sub_status: dict[str, Subscription] = {}  # CHECKME: which are useless?
        self.sub_status_v1: dict[str, Subscription] = {}  # CHECKME: needed?
        self.sub_status_v2: dict[str, Subscription] = {}  # CHECKME: needed?
        self.sub_status_v4: dict[str, Subscription] = {}  # CHECKME: needed?
        self.sub_position: dict[str, Subscription] = {}  # CHECKME: needed?
        self.sub_position_v1: dict[str, Subscription] = {}  # CHECKME: needed?
        self.sub_attitude: dict[str, Subscription] = {}
        self.sub_ang_vel: dict[str, Subscription] = {}

        # Data structures for state measurements
        self.status: dict[str, VehicleStatus] = {}
        self.position: dict[str, VehicleLocalPosition] = {}
        self.attitude: dict[str, VehicleAttitude] = {}
        self.angular_velocity: dict[str, VehicleAngularVelocity] = {}

        # Braid MPC settings
        # TODO: consider storing settings in config file
        self.dt = 0.2
        self.N = 20

        # Load data from yaml file
        pkg_share = FilePath(get_package_share_directory("px4_braid_mpc"))
        with open(pkg_share / "config" / "sim_params.yaml", "r", encoding="utf-8") as f:
            self.params = yaml.safe_load(f)
        self.namespaces = self.params["namespaces"]

        # Define Quality of Service profiles for publishers and subscribers
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )

        # Create listeners and publishers for state measurements and trajectories
        # NOTE: topic names are defined as absolute paths with a leading slash since the
        # robot name is already prepended to the topic name.
        self.subscribers = {}
        self.reference_traj_pub = {}
        for ns in self.namespaces:
            # Create trajectory publisher for MPC reference trajectory
            self.reference_traj_pub[ns] = self.create_publisher(
                MultiDOFJointTrajectory,
                f"/{ns}/reference_trajectory",
                qos_profile_pub,
            )

            # Create publishers for rviz visualization
            # To be displayed in combination with the predicted path published by the
            # PX4-MPC node, which is published on /{ns}/px4_mpc/predicted_path.
            self.reference_path_pub[ns] = self.create_publisher(
                Path,
                f"/{ns}/reference_path",
                10,
            )

            # Create subscribers for state measurements
            # NOTE: using partial to pass the namespace to the callback function, since
            # the subscriber expects a callback with a single argument of the type
            # callback(msg: MessageType) -> None.
            self.sub_status[ns] = self.create_subscription(
                VehicleStatus,
                f"/{ns}/fmu/out/vehicle_status",
                partial(self._status_callback, ns),
                qos_profile_sub,
            )
            self.sub_status_v1[ns] = self.create_subscription(
                VehicleStatus,
                f"/{ns}/fmu/out/vehicle_status_v1",
                partial(self._status_callback, ns),
                qos_profile_sub,
            )
            self.sub_status_v2[ns] = self.create_subscription(
                VehicleStatus,
                f"/{ns}/fmu/out/vehicle_status_v2",
                partial(self._status_callback, ns),
                qos_profile_sub,
            )
            self.sub_status_v4[ns] = self.create_subscription(
                VehicleStatus,
                f"/{ns}/fmu/out/vehicle_status_v4",
                partial(self._status_callback, ns),
                qos_profile_sub,
            )
            self.sub_position[ns] = self.create_subscription(
                VehicleLocalPosition,  # also includes linear velocity
                f"/{ns}/fmu/out/vehicle_local_position",
                partial(self._position_callback, ns),
                qos_profile_sub,
            )
            self.sub_position_v1[ns] = self.create_subscription(
                VehicleLocalPosition,
                f"/{ns}/fmu/out/vehicle_local_position_v1",
                partial(self._position_callback, ns),
                qos_profile_sub,
            )
            self.sub_attitude[ns] = self.create_subscription(
                VehicleAttitude,
                f"/{ns}/fmu/out/vehicle_attitude",
                partial(self._attitude_callback, ns),
                qos_profile_sub,
            )
            self.sub_ang_vel[ns] = self.create_subscription(
                VehicleAngularVelocity,
                f"/{ns}/fmu/out/vehicle_angular_velocity",
                partial(self._ang_vel_callback, ns),
                qos_profile_sub,
            )

    def step_callback(self):
        """
        Performs one iteration of the MPC loop, which includes the following steps:
            1. Read current state of each robot (e.g. from /odom topic)
            2. Update the traveled trajectories and the executed winding numbers
            3. Estimate the current progress variable tau for each robot
            4. Share state information and tau estimates with other robots
            5. Estimate global progress variable tau from local tau estimates
            6. Update MPC cost weights based on the current tau values
            7. Solve the MPC optimization problem to compute the optimal trajectory for
               each robot based on others' predicted trajectories
            8. Publish the computed trajectory to the corresponding topic for each robot
               (execution is taken care of by the PX4-MPC nodes running for each robot)

        Args:
            None

        Returns:
            None
        """
        # TODO: complete implementation
        trajectories = [np.zeros((10, 13)) for _ in self.namespaces]  # dummy trajectory

        # 8. publish trajectory to corresponding topic for each robot
        # 8a. interpolate trajectory to match horizon and time step of PX4-MPC
        interpolated_trajectories = [
            interpolation.interpolate_trajectory(
                trajectory=trajectories[i],
                timestep=self.dt,
                target_horizon=10,  # TODO: extract from px4-mpc
                target_timestep=0.1,  # TODO: extract from px4-mpc
            )
            for i, _ in enumerate(self.namespaces)
        ]

        # 8b. publish trajectory
        messages.publish_trajectories(
            trajectories=interpolated_trajectories,
            times=0.1,
            namespaces=self.namespaces,
            publishers=self.reference_traj_pub,
        )

    def _status_callback(self, ns: str, msg: VehicleStatus) -> None:
        # TODO: implement
        # self.status[ns] = msg
        pass

    def _position_callback(self, ns: str, msg: VehicleLocalPosition) -> None:
        # TODO: implement
        # self.position[ns] = msg
        pass

    def _attitude_callback(self, ns: str, msg: VehicleAttitude) -> None:
        # TODO: implement
        # self.attitude[ns] = msg
        pass

    def _ang_vel_callback(self, ns: str, msg: VehicleAngularVelocity) -> None:
        # TODO: implement
        # self.angular_velocity[ns] = msg
        pass
