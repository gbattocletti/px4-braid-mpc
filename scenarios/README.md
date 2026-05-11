# Scenarios launch scripts

Scripts to launch the simulation environment. The scripts launch the following components:
- A gazebo simulation (potentially headless)
- Three instances of the px4 flight controllers (one for each robot)

Example commands for the simulation launch script:
```sh
./launch_braid_sim.py                 # GUI gazebo
./launch_braid_sim.py --headless      # no gazebo GUI
./launch_braid_sim.py --kill          # tear down
byobu attach -t braid_sim             # attach
```