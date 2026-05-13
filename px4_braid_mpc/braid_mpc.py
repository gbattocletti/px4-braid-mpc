"""
Braid MPC node for PX4-based robots in the ATMOS simulator.
"""

from functools import partial
from pathlib import Path as FilePath

import numpy as np
import yaml
from ament_index_python import get_package_share_directory
from nav_msgs.msg import Odometry, Path
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
from visualization_msgs.msg import Marker

import px4_braid_mpc.messages as messages


class BraidMPC(Node):
    def __init__(self):
        super().__init__("braid_mpc")

        # Parameters and namespaces
        self.params: dict
        self.namespaces: list[str]

        # Publishers and listeners
        # TODO: check what topics are actually used and remove the unnecessary ones
        # (also remove the corresponding subscribers below)
        self.publishers: dict[str, Publisher]
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
        self.publishers = {}
        for ns in self.namespaces:
            # Create trajectory publisher for MPC reference trajectory
            self.publishers[ns] = self.create_publisher(
                MultiDOFJointTrajectory,
                f"/{ns}/px4_mpc/reference_trajectory",
                qos_profile_pub,
            )

            # Create publisher for rviz visualization
            self.predicted_path_pub = self.create_publisher(
                Path,
                f"/{ns}/px4_mpc/predicted_path",
                10,
            )
            self.reference_pub = self.create_publisher(
                Marker,
                f"/{ns}/px4_mpc/reference",
                10,
            )
            self.odom_pub = self.create_publisher(
                Odometry,
                f"/{ns}/odom",
                qos_profile_pub,
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

    def step(self):
        # TODO: complete implementation
        # 1. read current state of each robot (e.g. from /odom topic)
        # 2. update real trajectories + real winding numbers
        # 3. share state information between robots (e.g. via shared memory as the three
        #   robots are running in the same process, i.e. this node)
        # 4. estimate current progress variable tau for each robot + global one
        # 5. update weights for MPC cost function
        # 6. solve MPC optimization problem to compute optimal trajectory for each robot

        # 7. publish trajectory to corresponding topic for each robot
        #   --> execution is taken care of by the PX4-MPC nodes running for each robot
        trajectories = [np.zeros((10, 13)) for _ in self.namespaces]  # dummy trajectory
        messages.publish_trajectories(
            trajectories=trajectories,
            times=0.1,
            namespaces=self.namespaces,
            publishers=self.publishers,
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
