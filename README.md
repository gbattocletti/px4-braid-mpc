# Simulation of a braid MPC controller on a set of ATMOS robots

Simulation of the [MPC-based distributed braid controller](https://github.com/gbattocletti/braid-controller) on a multi-agent space-based tethered robot system. The simulation is based on the [DISCOWER ATMOS](https://atmos.discower.io/) platform, which is also used for the real-world experiments.


## Usage
A set of [launch scripts](launch/) are made available to launch all the required components to simulate the motion of the spacecrafts. 
To launch the simulation, first launch the mircoxrce dds bridge:
```sh
micro-xrce-dds-agent udp4 -p 8888
```
Then, launch the simulation and PX4 flight controllers from the launch scripts with:
```sh
cd ~/px4_ws/src/px4_braid_mpc/scenarios
./launch_sim.py
```
Finally, launch the mpc and controller:
```sh
ros2 launch px4_braid_mpc controller.launch.py
```
The most convenient way to arm the spacecrafts and start the simulation is to then open QGroundControl and manually set the flight mode to `offboard`, then arm the flyers.

Optionally, an rviz session for visualization of the reference frames can be launched with:
```sh
ros2 launch px4_braid_mpc rviz.launch.py
```
More details on the launch scripts can be found in the [launch folder](launch/README.md).


## Installation
The project has quite a few dependencies, such as ros2, gazebo, microxrce, and QGroundControl. Instructions for the complete setup of the control and simulation stack can be found on the original repositories of the submodules (best place to start is [here](https://atmos.discower.io/pages/Simulation/)), or in this [notes file](docs/dependencies-setup.md). 

To be built, the repo expects the following packages to be already cloned in the
`px4_ws/src/` folder:
- [`px4_msgs`](https://github.com/PX4/px4_msgs)
- [`px4-mpc`](https://github.com/gbattocletti/px4-mpc) 
- [`px4-world-publisher`](https://github.com/gbattocletti/px4-world-publisher)

Clone this repo and install the python submodules:
```sh
cd ~/px4_ws/src
git clone --recurse-submodules git@github.com:gbattocletti/px4-braid-mpc.git
pip install -e px4-braid-mpc/external/braid-controller
git config submodule.recurse true  # recommended to keep submodules in sync when pulling
```
Note that there are two submodules, but `px4-launch` is used as plain scripts and does not need to be installed. `braid-controller` is a pip package, which needs to be installed.

Finally, build the whole workspace:
```sh
cd ~/px4_ws
colcon build --symlink-install  # --packages-select px4_braid_mpc for incremental build
source install/setup.bash
```


## Repository structure
The repository is structured as follows:
```
px4_braid_mpc/
    │
    ├── package.xml                         # ROS package metadata
    ├── setup.py                            # ament_python build
    ├── setup.cfg
    ├── README.md
    ├── LICENSE
    ├── .pylintrc
    ├── .gitmodules
    ├── .gitignore
    │
    ├── resource/
    │   └── px4_braid_mpc                   # empty marker file
    │
    ├── px4_braid_mpc/                      # python package (ros2 node)
    │   ├── __init__.py
    │   ├── braid_mpc.py
    │   └── (TODO: helpers)
    │
    ├── config/
    │   ├── config.rviz
    │   └── sim_params.yaml                 # names, world, initial and final states
    │
    ├── scenarios/
    │   └── launch_sim.py                   # 3 x px4 + gazebo
    │
    ├── launch/                             # ros launch scripts
    │   ├── controller.launch.py            # 3 x px4-mpcs + high-level-controller 
    │   └── rviz.launch.py                  # px4-world-publisher + rviz
    │
    └── external/
        ├── px4-launch/                     # custom fork of px4-launch
        └── braid-controller/               # braid-controller repo 
```


## License
The repository is provided under the GNU GPLv3 License. See the LICENSE file included with this repository.

## Author
[Gianpietro Battocletti](https://www.tudelft.nl/staff/g.battocletti/), PhD Candidate at the [Delft Center for Systems and Control](https://www.tudelft.nl/en/me/about/departments/delft-center-for-systems-and-control/), [Delft University of Technology](https://www.tudelft.nl/en/).<br>
Contact information: [g.battocletti@tudelft.nl]().<br>
Copyright (c) 2026 Gianpietro Battocletti.