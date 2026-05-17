"""
Braid MPC node for PX4-based robots in the ATMOS simulator.
"""

from functools import partial
from pathlib import Path as FilePath

import numpy as np
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from braid_controller.core.agent import Agent
from braid_controller.core.mpc_distributed import DistributedMPC
from braid_controller.utils import invariants
from main import DT
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.service import Service
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from std_srvs.srv import Trigger
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

    Note: the braid MPC algorithm is designed to be implemented with a distributed
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

        ### LOAD SIMULATION DATA #######################################################
        # Simulation parameters
        self.params: dict  # general simulation parameters
        self.specification: dict  # parameters related to topological specification
        self.namespaces: list[str]  # list of namespaces of the robots

        # Load data from yaml file
        pkg_share = FilePath(get_package_share_directory("px4_braid_mpc"))
        with open(pkg_share / "config" / "sim_params.yaml", "r", encoding="utf-8") as f:
            self.params = yaml.safe_load(f)
        with open(
            pkg_share / "config" / self.params["specification"], "r", encoding="utf-8"
        ) as f:
            self.specification = yaml.safe_load(f)

        # Extract information from the loaded parameters
        self.namespaces = self.params["namespaces"]
        self.m = self.specification["m"]  # number of robots
        if self.m != len(self.namespaces):
            raise ValueError(
                f"Number of robots in the specification (m={self.specification['m']}) "
                f"does not match the number of namespaces defined in the parameters "
                f"(namespaces={self.params['namespaces']})."
            )

        ### CREATE PUBLISHERS AND SUBSCRIBERS ##########################################
        # Initialize publishers and listeners dictionaries
        self.reference_traj_pub: dict[str, Publisher] = {}
        self.reference_path_pub: dict[str, Publisher] = {}
        self.sub_status: dict[str, Subscription] = {}
        self.sub_status_v1: dict[str, Subscription] = {}
        self.sub_status_v2: dict[str, Subscription] = {}
        self.sub_status_v4: dict[str, Subscription] = {}
        self.sub_position: dict[str, Subscription] = {}
        self.sub_position_v1: dict[str, Subscription] = {}

        # Define Quality of Service profiles for publishers and subscribers
        qos_profile_pub: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )
        qos_profile_sub: QoSProfile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0,
        )

        # Create listeners and publishers for state measurements and trajectories
        # NOTE: topic names are defined as absolute paths with a leading slash since the
        # robot name is already prepended to the topic name.
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

        # Initialize data structures for state measurements
        self.status_timestamp: dict[str, float] = {}
        self.status_nav_state: dict[str, int] = {
            ns: VehicleStatus.NAVIGATION_STATE_MAX for ns in self.namespaces
        }
        self.status_arm_state: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.position_timestamp: dict[str, float] = {
            ns: -np.inf for ns in self.namespaces
        }
        self.position: dict[str, np.ndarray] = {
            ns: np.zeros(3, dtype=np.float32) for ns in self.namespaces
        }
        self.velocity: dict[str, np.ndarray] = {
            ns: np.zeros(3, dtype=np.float32) for ns in self.namespaces
        }

        ### SETUP AGENTS and CONTROLLER ################################################
        # Create agents
        self.M: list[Agent] = [Agent(i) for i in range(self.m)]
        for agent in self.M:
            agent.dt = DT
            agent.x = 0  # TODO
            agent.x_goal = 0  # TODO
            agent.x_opt = 0  # TODO
            agent.u_opt = 0  # TODO
            agent.t_sol = np.inf
            agent.cost = np.inf
            agent.cost_g = np.inf
            agent.cost_u = np.inf
            agent.cost_w = np.inf

        # Create distributed braid MPC controller
        self.controller: DistributedMPC = DistributedMPC(dynamics="single_integrator")
        self.controller.m = self.m
        self.controller.dt = self.params["dt"]  # timestep of braid MPC
        self.controller.K = self.params["N"]  # horizon of braid MPC
        self.controller.alpha_u = self.params["alpha_u"]  # weight for control cost
        self.controller.w_epsilon = self.params["w_epsilon"]  # winding constraint
        self.controller.d_min = self.params["d_min"]  # minimum safety distance
        self.controller.x_min = np.array(
            [
                self.params["x_lims"][0],
                self.params["y_lims"][0],
            ]
        )
        self.controller.x_max = np.array(
            [
                self.params["x_lims"][1],
                self.params["y_lims"][1],
            ]
        )
        # TODO: update values! Also, check if worth adding rate constraint
        self.controller.u_min = np.array([-1, -1])  # u is a vector [v_x, v_y]
        self.controller.u_max = np.array([1, 1])
        self.controller.R = np.diag([1, 1])
        self.controller.initialize_ocp()

        # Other parameters for braid MPC controller
        self.alpha_g: float = self.params["alpha_g"]  # weight for progress cost
        self.alpha_exp: float = self.params["alpha_exp"]  # exponent for goal weight
        self.alpha_w: float = self.params["alpha_w"]  # weight for winding cost
        self.consensus: str = self.params["consensus"]  # consensus type for tau

        # TODO: continue controller initialization

        # Settings for the PX4-MPC nodes
        # NOTE: data copied from the SpacecraftWrenchMPC class in the
        # px4_mpc/controllers/spacecraft_wrench_mpc.py file of the px4-mpc package.
        self.dt_px4_mpc: float = 5.0 / 29
        self.N_px4_mpc: int = 30

        # Gate flag (must be True to publish the reference trajectories)
        self.publishing_enabled: bool = False

        # Advertise the trigger service
        self.start_service: Service = self.create_service(
            Trigger,
            "~/start",  # ~/ makes it namespace-relative: /high_level_controller/start
            self._start_callback,
        )

        # Print initialization completion message
        self.get_logger().info(
            "High-level controller ready. "
            "Call /high_level_controller/start to begin publishing."
        )

        ### PREPROCESS TOPOLOGICAL SPECIFICATION #######################################
        # TODO: consider moving to dedicated method
        self.grids: np.ndarray = self.specification["grids"]
        self.n_generators: int = self.grids.shape[0]

        # Compute target winding numbers
        paths = invariants.grids2paths(self.grids)
        self.w_target: np.ndarray = invariants.paths2windings(
            paths,
            upscale_factor=1000,
            intermediate_shape="linear",
        )  # (n_generators * upscale_factor, m, m)
        self.n_windings: int = self.w_target.shape[0]  # length of w_target

        ### CREATE MAIN CONTROL CALLBACK ###############################################
        # Initialize timer for main control loop
        timer_period: float = 0.2  # seconds
        self.timer: Timer = self.create_timer(
            timer_period,
            self._control_step_callback,
        )

    def _control_step_callback(self):
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
        # Check data validity
        if not self._check_data_validity():
            return
        elif not self.publishing_enabled:
            self.get_logger().info(
                "Data is valid but publishing is not enabled. Waiting for user to "
                "trigger the start service "
                "(ros2 service call /high_level_controller/start std_srvs/srv/Trigger)."
            )
            return

        # TODO: complete implementation
        trajectories = [np.zeros((10, 13)) for _ in self.namespaces]  # dummy trajectory

        # 8. publish trajectory to corresponding topic for each robot
        # 8a. interpolate trajectory to match horizon and time step of PX4-MPC
        interpolated_trajectories = [
            interpolation.interpolate_trajectory(
                trajectory=trajectories[i],
                timestep=self.controller.dt,
                target_horizon=self.N_px4_mpc,
                target_timestep=self.dt_px4_mpc,
            )
            for i, _ in enumerate(self.namespaces)
        ]

        # 8b. publish reference trajectories to px4-mpc and reference paths to rviz
        for i, ns in enumerate(self.namespaces):

            # Build messages from trajectory of i-th robot
            reference_traj_msg: MultiDOFJointTrajectory = (
                messages.build_trajectory_message(
                    trajectory=trajectories[i],
                    times=self.dt_px4_mpc,  # CHECKME
                )
            )
            reference_path_msg: Path = messages.build_path_messages(
                trajectory=interpolated_trajectories[i],
                clock=self.get_clock().now().to_msg(),
            )

            # Publish messages
            self.reference_traj_pub[ns].publish(reference_traj_msg)
            self.reference_path_pub[ns].publish(reference_path_msg)

    def _status_callback(self, ns: str, msg: VehicleStatus) -> None:
        """
        Callback function for handling status messages from each robot.

        Args:
            ns (str): Namespace of the robot.
            msg (VehicleStatus): Status message from the robot.

        Note: the callback is copied from that defined in the mpc_spacecraft node of the
        px4_mpc package, with minor modifications.
        """
        self.status_timestamp[ns] = self.get_clock().now().nanoseconds / 1e9
        self.status_nav_state[ns] = msg.nav_state
        self.status_arm_state[ns] = msg.arming_state

    def _position_callback(self, ns: str, msg: VehicleLocalPosition) -> None:
        """
        Position callback function for handling local position messages from each robot.

        Args:
            ns (str): Namespace of the robot.
            msg (VehicleLocalPosition): Local position message from the robot, which
                includes both position and linear velocity in the local frame.

        Note: the callback is copied from that defined in the mpc_spacecraft node of the
        px4_mpc package, with minor modifications.
        Note: the callback performs a NED to ENU transformation, since the local
        position messages from the fmu is expressed in a NED frame, while the MPC
        controller works with a global ENU frame.
        """
        self.position_timestamp[ns] = self.get_clock().now().nanoseconds / 1e9
        self.position[ns][0] = msg.y
        self.position[ns][1] = msg.x
        self.position[ns][2] = -msg.z
        self.velocity[ns][0] = msg.vy
        self.velocity[ns][1] = msg.vx
        self.velocity[ns][2] = -msg.vz

    def _check_data_validity(self) -> bool:
        """
        Check that the robots' status and position data are recent enough to be used for
        trajectory planning. The method is adapted from SpacecraftMPC node in px4-mpc.
        """
        # Define thresholds for data validity (in seconds)
        DATA_VALIDITY_STREAM = 0.5
        DATA_VALIDITY_STATUS = 2.0

        # Initialize default return data
        data_is_valid = True

        # Check if the data is valid based on the timestamps
        current_time = self.get_clock().now().nanoseconds / 1e9
        for ns in self.namespaces:
            if current_time - self.position_timestamp[ns] > DATA_VALIDITY_STREAM:
                self.get_logger().warn(
                    "Vehicle position data is too old. Skipping trajectory planning."
                )
                data_is_valid = False
            if current_time - self.status_timestamp[ns] > DATA_VALIDITY_STATUS:
                self.get_logger().warn(
                    f"Vehicle status for robot {ns} is too old. "
                    "Skipping trajectory planning."
                )
                data_is_valid = False

        return data_is_valid

    def _start_callback(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """
        Callback function for the start service. Sets the gate flag to True, allowing
        the reference trajectories to be published in the control loop.

        To trigger the service, run the following command in the terminal:
            ros2 service call /high_level_controller/start std_srvs/srv/Trigger

        Args:
            request (Trigger.Request): Service request (not used).
            response (Trigger.Response): Service response, with success flag <- True.

        Returns:
            Trigger.Response: Service response with success flag set to True.
        """
        if self.publishing_enabled:
            response.success = False
            response.message = "Already publishing."
        else:
            self.publishing_enabled = True
            response.success = True
            response.message = "Trajectory publishing started."
            self.get_logger().info("Received start command. Publishing enabled.")
        return response


# Node entry point when executed directly
def main(args=None):
    rclpy.init(args=args)

    spacecraft_mpc = BraidMPC()

    rclpy.spin(spacecraft_mpc)

    spacecraft_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
