#!/usr/bin/env python3
"""
Dummy reference trajectory publisher for integration testing.

Publishes a constant `MultiDOFJointTrajectory` reference for each of m robots on the
topic `<namespace>/px4_mpc/reference_trajectory`, where the namespace is one of
`/px1`, `/px2`, `/px3` by default. Each robot is given a different hover setpoint so
that the receiving controllers can be visually distinguished.

This script is meant as a stand-in for the high-level distributed MPC controller from
`braid-mpc`. It allows the low-level spacecraft MPC (subscribing to
`<namespace>/px4_mpc/reference_trajectory`) to be tested in isolation while the
upstream planner is still under development.

Run:
    chmod +x test_trajectory_publisher.py  # only once
    ./test_trajectory_publisher.py
or:
    python3 test_trajectory_publisher.py

Then check the topics:
    ros2 topic echo /atmos_1/px4_mpc/reference_trajectory

Then, a subscriber from px4-mpc can be launched to check it is able to receive and parse
the message:
    ros2 launch px4_mpc mpc_spacecraft_launch.py \
    mode:=wrench namespace:=atmos_1 setpoint_from_rviz:=false

Optionally, a temporary logger can be added in `px4-mpc/mpc_spacecraft.py` at the end of 
`_reference_trajectory_callback`:
    self.get_logger().info(
        f"Trajectory received: pos[:, 0]={self.trajectory_position[:, 0]}, "
        f"att[:, 0]={self.trajectory_attitude[:, 0]}",
        throttle_duration_sec=1.0,
    )

Lastly, a SITL simulation can be launched to check the whole pipeline, including the 
px4-mpc output to the PX4 SITL.
"""

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Transform, Twist
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# Configuration -----------------------------------------------------------------------

# Match the controller's prediction horizon. The receiver expects exactly N+1 points.
N = 29  # so the trajectory has N+1 = 30 points
DT = 5.0 / N  # time step between trajectory points (seconds), matches Tf=5.0

# Per-robot hover setpoints: namespace -> (x, y, z, qw, qx, qy, qz).
# Different positions make the three subscribers visually distinguishable.
SETPOINTS = {
    "atmos_1": (1.0, 0.5, 0.0, 0.0, 0.0, 0.0, 1.0),
    # "atmos_2": (1.4, -0.8, 0.0, 0.0, 0.0, 0.0, 1.0),
    # "atmos_3": (1.4, 0.2, 0.0, 0.0, 0.0, 0.707, 0.707),
}

# Publishing rate (Hz). The reference trajectory does not need to be high-rate, since
# it covers a horizon of Tf seconds into the future and is only consumed by the MPC
# at its own rate.
PUBLISH_RATE_HZ = 5.0


# Node --------------------------------------------------------------------------------


class TestTrajectoryPublisher(Node):
    """
    Publish a constant hover trajectory for each robot at a fixed rate.
    """

    def __init__(self) -> None:
        super().__init__("test_trajectory_publisher")

        # One publisher per namespace
        self.publishers_by_ns = {
            ns: self.create_publisher(
                MultiDOFJointTrajectory,
                f"/{ns}/reference_trajectory",
                10,
            )
            for ns in SETPOINTS
        }

        # Pre-build the message for each namespace (constant in time, so no need to
        # rebuild on every callback — only the header stamp needs updating).
        self.messages_by_ns = {
            ns: self._build_constant_trajectory(setpoint)
            for ns, setpoint in SETPOINTS.items()
        }

        # Periodic publishing
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._timer_callback)

        self.get_logger().info(
            f"Publishing constant reference trajectories on "
            f"{list(self.publishers_by_ns.keys())} at {PUBLISH_RATE_HZ} Hz "
            f"({N + 1} points per message, dt={DT:.3f}s)."
        )

    def _build_constant_trajectory(
        self, setpoint: tuple[float, float, float, float, float, float, float]
    ) -> MultiDOFJointTrajectory:
        """
        Build a MultiDOFJointTrajectory with N+1 identical points at the given pose,
        zero velocity and angular velocity.
        """
        x, y, z, qw, qx, qy, qz = setpoint

        # Pose (constant over the horizon)
        transform = Transform()
        transform.translation.x = x
        transform.translation.y = y
        transform.translation.z = z
        transform.rotation.w = qw
        transform.rotation.x = qx
        transform.rotation.y = qy
        transform.rotation.z = qz

        # Twist (zero for a hover setpoint)
        twist = Twist()  # all fields default to 0.0

        # Assemble N+1 identical points, each with a time offset along the horizon
        msg = MultiDOFJointTrajectory()
        msg.joint_names = ["spacecraft"]  # single "joint" = the rigid body
        for i in range(N + 1):
            point = MultiDOFJointTrajectoryPoint()
            point.transforms.append(transform)
            point.velocities.append(twist)
            # accelerations left empty (the receiver does not use them)

            # time_from_start: ROS Duration, computed as i * DT
            total_ns = int(i * DT * 1e9)
            point.time_from_start = Duration(
                sec=total_ns // 1_000_000_000,
                nanosec=total_ns % 1_000_000_000,
            )
            msg.points.append(point)

        return msg

    def _timer_callback(self) -> None:
        """
        Publish the pre-built message for each namespace, refreshing the header stamp.
        """
        stamp = self.get_clock().now().to_msg()
        for ns, msg in self.messages_by_ns.items():
            msg.header.stamp = stamp
            msg.header.frame_id = "map"
            self.publishers_by_ns[ns].publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TestTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
