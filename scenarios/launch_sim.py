#!/usr/bin/env python3
"""
Scenario: 3-robot simulation.

Starts 3 atmos free-flyers in the kthspacelab world with their respective namespaces and
px4 SITL instances. The world is launched in Gazebo, and the robots are spawned in it.
The robots instances are launched in a byobu session named "atmos".

Use --headless to start Gazebo without its GUI (useful when you want
to visualize through rviz instead). Use --kill to tear it all down.

NOTE: this file needs to be made executable (chmod +x launch_sim.py) to be run directly.
Otherwise, it can be run with "python3 launch_sim.py" from the command line.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "external" / "px4-launch"))
from px4_sitl_launcher import launch, Vehicle

VEHICLES = [
    Vehicle(name="atmos_1", model="gz_atmos", pose=(1, 0, 1, 0, 0, 0)),
    Vehicle(name="atmos_2", model="gz_atmos", pose=(2, 0, 1, 0, 0, 0)),
    Vehicle(name="atmos_3", model="gz_atmos", pose=(3, 0, 1, 0, 0, 0)),
]

if __name__ == "__main__":
    launch(
        vehicles=VEHICLES,
        world="kthspacelab",
        session="atmos",
        multiplexer="byobu",  # multiplexer backend ("tmux" or "byobu"),
        headless=False,  # CLI --headless / --no-headless overrides this,
    )
