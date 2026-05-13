from typing import Sequence

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, Transform, Twist, Vector3
from rclpy.publisher import Publisher
from std_msgs.msg import Header
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


def build_trajectory_message(
    trajectory: np.ndarray,
    times: Sequence[float] | float,
    frame_id: str = "map",
    stamp=None,
) -> MultiDOFJointTrajectory:
    """Build a MultiDOFJointTrajectory from a numpy array.

    Args:
        trajectory: shape (N+1, 13) or (N+1, 16).
            M == 13: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
            M == 16: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, Fx, Fy, Tz]
        times: either a scalar dt (uniform sampling, t_k = k*dt) or a
            sequence of length N+1 giving time_from_start in seconds.
        frame_id: reference frame for the trajectory.
        stamp: optional rclpy time; if omitted, header.stamp is zeroed.

    Returns:
        MultiDOFJointTrajectory ready to publish.
    """
    # Validate inputs
    n, m = trajectory.shape  # NOTE: n = N+1, m = nx or nx+nu
    if m not in (13, 16):
        raise ValueError(f"Unsupported state width {m}; expected 13 or 16.")

    # Resolve times
    if np.isscalar(times):
        times = [k * float(times) for k in range(n)]
    elif len(times) != n:
        raise ValueError(f"len(times)={len(times)} != N={n}")

    # Initialize message
    msg = MultiDOFJointTrajectory()
    msg.header = Header(frame_id=frame_id)
    if stamp is not None:
        msg.header.stamp = stamp
    msg.joint_names = ["base"]

    # Build trajectory as sequence of MultiDOFJointTrajectoryPoint
    for i in range(n):
        point = MultiDOFJointTrajectoryPoint()

        # Pose
        tf = Transform()
        tf.translation = Vector3(
            x=float(trajectory[i, 0]),
            y=float(trajectory[i, 1]),
            z=float(trajectory[i, 2]),
        )
        tf.rotation = Quaternion(
            w=float(trajectory[i, 3]),
            x=float(trajectory[i, 4]),
            y=float(trajectory[i, 5]),
            z=float(trajectory[i, 6]),
        )
        point.transforms = [tf]

        # Velocities
        tw = Twist()
        tw.linear = Vector3(
            x=float(trajectory[i, 7]),
            y=float(trajectory[i, 8]),
            z=float(trajectory[i, 9]),
        )
        tw.angular = Vector3(
            x=float(trajectory[i, 10]),
            y=float(trajectory[i, 11]),
            z=float(trajectory[i, 12]),
        )
        point.velocities = [tw]

        # Controls (if present)
        if m == 16:
            # NOTE: we pack controls into the accelerations field of the message,
            # which is a bit hacky but allows us to reuse the same message type.
            # Mapping: (Fx, Fy, Tz) -> (ax, ay, az)
            point.accelerations = [
                Vector3(
                    x=float(trajectory[i, 13]),
                    y=float(trajectory[i, 14]),
                    z=float(trajectory[i, 15]),
                )
            ]

        # Time (from start of trajectory)
        t = float(times[i])
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        # Append point to trajectory message
        msg.points.append(point)

    return msg


def publish_trajectories(
    trajectories: list[np.ndarray],
    times: Sequence[float] | float,
    namespaces: Sequence[str],
    publishers: dict[str, Publisher],
):
    """
    Publish the trajectories to the corresponding publishers.

    Args:
        trajectories (list[np.ndarray]): list of N+1 x 13 or N+1 x 16 numpy arrays,
            one per robot.
        namespaces (Sequence[str]): list of robot namespaces, one per robot.
        publishers (dict[str, Publisher]): dict mapping each namespace to the
            corresponding Publisher for that robot.

    Returns:
        None
    """
    for i, ns in enumerate(namespaces):
        trajectory: MultiDOFJointTrajectory = build_trajectory_message(
            trajectory=trajectories[i],
            times=times,
        )
        publishers[ns].publish(trajectory)
