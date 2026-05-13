import numpy as np


def interpolate_trajectory(
    trajectory: np.ndarray,
    timestep: float,
    target_horizon: int,
    target_timestep: float,
) -> np.ndarray:
    """
    Interpolate the given trajectory to a different horizon and timestep.

    Args:
        trajectory (np.ndarray): N+1 x M array, where N is the original horizon and M
            is the number of state/control
        timestep (float): original time step (seconds)
        target_horizon (int): desired horizon (number of time steps)
        target_timestep (float): desired time step (seconds)

    Returns:
        np.ndarray: target_horizon+1 x M array, interpolated trajectory
    """
    raise NotImplementedError("TODO: implement interpolation logic")
