# Simulation of a braid MPC controller on a set of ATMOS robots

## Install
The project has quite a bit of dependencies, such as ros2, gazebo, urxce, and qgroundcotnrol. Instructions for the complete setup of the control and simulation stack can be found on the original repositories of the submodules, or [here](dev/dependencies-setup.md). 

To be built, the repo expects the following packages are also in your
`px4_ws/src/`:
- [`px4_msgs`](https://github.com/PX4/px4_msgs)
- [`px4_mpc`](https://github.com/gbattocletti/px4-mpc) 
- [`px4_-_world_publisher`](https://github.com/gbattocletti/px4-world-publisher)
Clone them before running the build step below:
```sh
cd ~px4_ws/src/
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/gbattocletti/px4-mpc.git
git clone https://github.com/gbattocletti/px4-world-publisher.git
```
Then clone this repo and install the python submodules:
```sh
git clone --recurse-submodules git@github.com:gbattocletti/px4-braid-mpc.git
pip install -e px4_braid_mpc/external/braid-controller
git config submodule.recurse true  # recommended to keep submodules in sync when pulling
```
Note that there are two submodules, but `px4-launch` is used as plain scripts and does not need to be installed. `braid-controller` is a pip package, which needs to be installed.

Finally, build the whole workspace:
```sh
cd ~/px4_ws
colcon build --symlink-install
source install/setup.bash
```


## Project structure
The project is structured as follows:
```
px4_braid_mpc/
    │
    ├── package.xml                         # ROS package metadata
    ├── setup.py                            # ament_python build
    ├── setup.cfg
    ├── pyproject.toml
    ├── README.md
    ├── LICENSE
    ├── .gitmodules
    ├── .gitignore
    │
    ├── resource/
    │   └── px4_braid_mpc                   # empty marker file
    │
    ├── px4_braid_mpc/                      # python package (the nodes live here)
    │   ├── __init__.py
    │   ├── high_level_controller.py        # entry point for the px4_braid_mpc node
    │   └── (any helpers)
    │
	├── scenarios/                          # gazebo simulation launchers
    │   └── launch_braid_sim.py             # 3 x px4 + gazebo
    │
    ├── launch/                             # ROS launch files
    │   └── braid_sim_launch.py             # 3 x px4-mpcs + high-level-controller (this repo) + px4-world-publisher + rviz
    │
    ├── config/
    │   ├── vehicles.yaml                   # names, poses, world
    │   └── braid.rviz
    │
    └── external/
        ├── px4-launch/                     # custom fork of px4-launch (for scenarios)
        └── braid-controller/               # braid-controller repo (for the high-level-controller)
```

## Launcher files
The repo provides two types of launch scripts: 
1. A script to launch the simulation environment, which is composed of:
	- A gazebo simulation (potentially headless)
	- Three px4 flight controllers (one for each robot)
2. A script to launch the controller stack, which is composed of:
	- Three px4-mpc nodes, acting as low-level controller and communicating directly with the px4 flight controllers  
	- A ros2 node for the high_level_controller, communicating with the px4 flight controllers and the px4-mpc nodes
	- A rviz session for a lightweight visualization of the simulation
In addition to these two files, the simulation stack requires the microrxce dds bridge to enable the exchange of data between the px4 flight controllers and ros2, and qgroundcontrol to arm the robots in offboard mode. 

Example commands for the simulation launch script:
```sh
./launch_braid_sim.py                 # GUI gazebo
./launch_braid_sim.py --headless      # no gazebo GUI
./launch_braid_sim.py --kill          # tear down
byobu attach -t braid_sim             # attach
```

Example commands for the controllers launch script:
```sh

```

Example of the complete simulation launch sequence:
```sh

```
