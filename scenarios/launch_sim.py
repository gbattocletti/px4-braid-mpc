#!/usr/bin/env python3
"""
Scenario: 3-robot simulation.

Starts 3 atmos free-flyers in the kthspacelab world with their respective namespaces and
px4 SITL instances. The world is launched in Gazebo, and the robots are spawned in it.
The robots instances are launched in a byobu session named "atmos".

Usage:
    cd ~/px4_ws/src/px4-braid-mpc/scenarios
    ./launch_sim.py
Use --headless to start Gazebo without its GUI. Use --kill to tear it all down.
NOTE: this file needs to be made executable (chmod +x launch_sim.py) to be run directly.
Otherwise, it needs to be run with "python3 launch_sim.py".
"""

import sys
from pathlib import Path

import yaml

root = Path(__file__).parent.parent
sys.path.insert(0, str(root / "external" / "px4-launch"))
from px4_sitl_launcher import Vehicle, launch

# Read config parameters
with open(root / "config" / "sim_params.yaml", "r", encoding="utf-8") as f:
    params = yaml.safe_load(f)

# Build list of vehicles to be launched in the simulation
VEHICLES = [
    Vehicle(
        name=ns,
        model="gz_atmos",
        pose=tuple(params["initial_states"][ns]),
    )
    for ns in params["namespaces"]
]

# Launch the simulation with the defined vehicles and parameters
if __name__ == "__main__":
    launch(
        vehicles=VEHICLES,
        world="kthspacelab",
        session="atmos",
        multiplexer="byobu-tmux",  # multiplexer backend ("tmux" or "byobu-tmux"),
        headless=True,  # CLI --headless / --no-headless overrides this,
    )
