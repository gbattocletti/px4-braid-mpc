# Controller launch scripts

Entry points for the controller nodes.

The repo provides two relevant launch scripts: 
1. `controller.launch.py`: launches the controller stack, which is composed of:
	- Three px4-mpc nodes, acting as low-level controller and communicating directly with the px4 flight controllers  
	- A ros2 node for the high_level_controller, communicating with the px4 flight controllers and the px4-mpc nodes
2. `rviz.launch.py`: starts a rviz session for a lightweight visualization of the simulation. The script also launches a `px4-world-publisher` node that allows displaying the world geometries in rviz

The controllers also need an instance of microrxce dds bridge to be able to exchange data with the px4 flight controllers (both in simulation and real world experiments).
Additionally, the robots must be set to offboard mode and armed via qgroundcontrol. 

Example commands for the controllers launch script:
```sh
ros2 launch px4_braid_mpc controller.launch.py
ros2 launch px4_braid_mpc rviz.launch.py
```
