# Braid sim

## Install
The project has quite a bit of dependencies, such as ros2, gazebo, urxce, and qgroundcotnrol. Instructions for the complete setup of the control and simulation stack can be found on the original repositories of the submodules, or [here](dev/simulation-setup.md). Finally, download the custom publisher of the `kthspacelab` geometries for visualization in rviz:
```sh
cd ~px4_ws/src/
git clone git@github.com:gbattocletti/px4-world-publisher.git
colcon build --packages-select px-world-publisher
```
After setting up all the required dependencies, clone and build the repo:
```sh
cd ~px4_ws/src/
git clone --recurse-submodules git@github.com:gbattocletti/px4-braid-mpc.git
pip install -e px4_braid_mpc/external/braid-controller
colcon build --packages-select px4_braid_mpc --symlink-install
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