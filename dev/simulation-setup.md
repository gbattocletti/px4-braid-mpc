# ATMOS simulation setup

This file contains a summary of the setup steps for the simulation and experimental stack to use the atmos robots, simulating their behavior in a gazebo simulation along with that of the px4 flight controller and controlling them via a set of ros2 nodes.
More information can be found on the official [DISCOWER ATMOS](https://atmos.discower.io/) website.


------------------------------------------------------------------------------
## General setup

### Python virtual environments
Some notes on the use of venvs with ROS (which are generally not supported and should be avoided):

#### Conda
Before installing ros, make sure conda is deactivated to avoid conflicts between the conda python and the system python (which is the one ROS uses). This can be achieved with:
```sh
conda deactivate
```
More in general, this should be applied permanently to the terminal to avoid any future conflict (which apparently are quite common when using ROS)
```sh
conda config --set auto_activate_base false
```

#### uv 
In general, it is better to just use the system python version for ros. A possible alternative to isolate at least partially the dependency stack from the system python version is to use a venv created with the `--system-site-packages` option to inherit the python version and existing dependencies from the system python, and that allows for the installation of additional packages without polluting the system python. uv can be installed by running:
```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
```
An uv venv inheriting the system python version and packages can be created with:
```sh
uv venv ~/px4_venv --python 3.10 --system-site-packages
```
Dependencies can then be added with:
```sh
uv pip install "numpy<2.0" scipy matplotlib casadi future-fstrings
cd $ACADOS_SOURCE_DIR/interfaces/acados_template
uv pip install -e .
cd /path/to/your/repo
uv pip install -e .  # this also installs all the dependencies of the repo
```
Last, the venv must be activated before launching ros. This can be achieved by adding the following to ~/.bashrc:
```sh
source ~/px4_venv/bin/activate
```
NOTE: in .bashrc this line should be AFTER sourcing the ros and px4 setup.bash files (so that the order in which PYTHONPATH is composed is correct), i.e.:
```sh
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
source ~/px4_venv/bin/activate  # goes last
```

#### docker
The best way to have an isolated python environment where to install all the dependencies is to create a dedicated docker container. [Several resources](https://docs.ros.org/en/rolling/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html) are available online for this. 


### Installation environment
The target version in this file is ROS2 Humble + Gazebo Fortress on Ubuntu 22.04.

For different target versions (e.g. ROS2 Jazzy + Gazebo), the steps must be adapted slightly.

#### Ubuntu 22.04
Everything can be installed regularly without additional steps.

#### Linux Mint
For installation on linux mint, setup Distrobox to create a containerized ubuntu environment.
- install distrobox https://forums.linuxmint.com/viewtopic.php?t=419822
```sh
sudo apt update
sudo apt upgrade
sudo apt install distrobox # podman automatically installed
```
- optionally, install BoxBuddy from FlatHub to manage boxes from GUI
- create a box for ROS Humble, either from boxbuddy or from terminal by running:
```sh
distrobox create --image ubuntu:22.04 --name ubuntu-22.04
```
- Enter the distrobox, either via boxbuddy or by running
```sh
distrobox enter <container>
```
Optionally, add an alias for the container (add to .bashrc) 
```sh
alias humble='distrobox enter <container>'
```
which in this case would read:
```sh
alias humble='distrobox enter ubuntu-22.04'
```

#### Windows via WSL
For installation on windows via wsl, install the desired distro in wsl https://learn.microsoft.com/en-us/windows/wsl/install
```sh
wsl --list --online
wsl --install <Distro>
wsl --set-default <Distro>  # optional
```

#### docker
Again, [docker](https://docs.ros.org/en/rolling/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html) is a suitable approach to create an Ubuntu 22.04 environment where to install everything.


### ssh agent setup for github
If this is not setup, all the links of the clone action of the type 
```sh
git clone git@github.com:PX4/PX4-Autopilot.git
```
must be replaced with 
```sh
git clone https://github.com/PX4/PX4-Autopilot.git
```
Alternatively, an ssh connection can be set up with github to be able to pull/push from github (mandatory for private repos). This can be achieved by running
```sh
ssh-keygen -t ed25519 -C "johndoe@email.com"
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
cat ~/.ssh/id_ed25519.pub  # copy output
```
Then open GitHub, go to Settings > SSH and GPG keys > New SSH key, give it a title and paste the output of the cat command above. Finally, test the connection from the terminal by running
```sh
ssh -T git@github.com
```
Lastly, register the user name and email to be able to push new commits:
```sh
git config --global user.name "johndoe"
git config --global user.email "johndoe@email.com"
```



-------------------------------------------------------------------------------
## ROS + Gazebo Installation

### ROS
Install ROS2 Humble:
```sh
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

# Enable required repositories
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS2 Humble including dev tools
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop ros-dev-tools -y

# Source ROS2
source /opt/ros/humble/setup.bash
```
NOTE: the source command must be executed every time a new terminal is opened. This can be avoided by adding the following line to .bashrc:
```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
To make the change take effect in the terminal where the command above is execute, source the .bashrc file `source ~/.bashrc`. 

To test the ros2 installation, one can try the classic talker-listener test. In 2 separate terminals, run:
```sh
ros2 run demo_nodes_cpp talker
```
and
```sh
ros2 run demo_nodes_py listener
```


### Gazebo
Install Gazebo Fortress
```sh
sudo apt install ros-humble-ros-gz -y
```
NOTE (only for distrobox): In a distrobox container it is necessary to set the ING_IP to localhost to avoid hostname issues with gazebo transport in distrobox. To fix execute the following from inside this the distrobox to set the IGN_IP to localhost every time the distrobox is launched
```sh
echo "export IGN_IP=127.0.0.1" >> ~/.bashrc
source ~/.bashrc
```

Test the gazebo installation
```sh
ign gazebo -v 4 shapes.sdf
```



------------------------------------------------------------------------------
## ATMOS

Official Discower website: https://atmos.discower.io/
Gazebo simulator guide: https://atmos.discower.io/pages/Simulation/


### PX4 flight controller
Install the PX4 flight controller https://github.com/PX4/PX4-Autopilot
```sh
git clone git@github.com:PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
It is suggested to reboot the machine at this point.

Build PX4 messages for ROS2
```sh
mkdir -p ~/px4_ws/src/
cd ~/px4_ws/src/
git clone git@github.com:PX4/px4_msgs.git
cd ~/px4_ws
colcon build
source install/setup.bash  # only has effect on current terminal
```
Make the sourcing of `setup.bash` permanent by adding the next line to `.bashrc`
```sh
source ~/px4_ws/install/setup.bash
```
Download the ATMOS dds_topics (https://atmos.discower.io/assets/px4_autopilot/dds_topics.yaml) and replace the ones that come with PX4, located under `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` (see point 4 at https://atmos.discower.io/pages/PX4/#px4-autopilot)

Ttest the px4 controller by moving to the PX4-Autopilot folder and running
```sh
make px4_sitl_spacecraft gz_atmos
```
NOTE: the PX4 flight controller is actually only needed for the simulation, since it is the one directly interfacing with the robot model in gazebo.


### QGroundControl
Prepare the system for the installation of QGroundControl (https://github.com/mavlink/qgroundcontrol)
```sh
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Ddownload QGroundControl from https://docs.qgroundcontrol.com/master/en/qgc-user-guide/releases/daily_builds.html#daily-builds 

Change the permissions and run it:
```sh
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage
```
Set up QGC as described at https://github.com/DISCOWER/px4-mpc#qgc-setup-or-headless-no-qgc-setup: start QGroundControl: from a new terminal execute `qgroundcontrol` to open the program, then go to settings (QGC icon in top-left corner) > fly view > Virtual Joystics = enabled, auto-center throttle = disabled > back to home. 

Optionally, extract the app to be able to create an alias and launch it from any location via the `qgroundcontrol` command.
```sh
chmod +x QGroundControl-x86_64.AppImage
./QGroundControl-x86_64.AppImage --appimage-extract
./squashfs-root/AppRun  # launches the app
mv squashfs-root/QGroundControl/  # change of folder name
echo '#!/bin/bash
exec ~/<path-to-QGroundControl/QGroundControl/AppRun "$@"' > ~/.local/bin/qgroundcontrol  # NOTE: copy from echo... until here all together + put the correct path in the exec line
chmod +x ~/.local/bin/qgroundcontrol
```
NOTE: the log folder `QGroundControl Daily` is placed in the `Documents` folder by default. This can be changed in the app settings from: Settings > General > Application Load/Save Path.

NOTE: in principle the QGC interface can be skipped in simulation, but is required for real world tests so it makes sense to include in the software-in-the-loop simulations. To skip QGC some px4 parameters need to be changed, as described at the bottom of https://github.com/DISCOWER/px4-mpc#qgc-setup-or-headless-no-qgc-setup.


### Micro-XRCE-DDS-Agent
Microxrce allows the px4 flight controller to communicate with ros2. It can be installed from https://micro-xrce-dds.docs.eprosima.com/en/latest/agent.html with
```sh
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build 
cd build
cmake ..
make -j$(nproc)  # may be redundant (to be checked)
sudo make install
sudo ldconfig /usr/local/lib/
```
Optionally, create an alias to launch MicroXRCEAgent from anywhere (may be mandatory if using a non-standard installation location)
```sh
echo 'alias microxrce="$HOME/<path-to-folder>/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"' >> ~/.bashrc  # note: select correct path after $HOME
source ~/.bashrc
``` 



-----------------------------------------------------------------------------
## Offboard PX4 control

In order to control the PX4 from a program running on the laptop (e.g. a python project) some additional steps are required (see https://github.com/DISCOWER/px4-mpc and https://github.com/DISCOWER/px4_launch)


### Acados
- install [acados](https://docs.acados.org/installation/) (required by the mpc in px4_mpc, can be skipped if that MPC module is not used)
```sh
cd ~/Documents/software
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build
cd build
cmake -DACADOS_WITH_OPENMP=ON -DACADOS_PYTHON=ON ..
make install -j4
```
add acados to LD_LIBRARY_PATH
```sh
export ACADOS_SOURCE_DIR=$HOME/Documents/software/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
```
Install the acados python interface https://docs.acados.org/python_interface/index.html
```sh
pip3 install -e $ACADOS_SOURCE_DIR/interfaces/acados_template
```
Test acados by running a test acados file, e.g., `<acados_root>/examples/acados_python/getting_started/minimal_example_ocp.py` (to be run after `cd` in the corresponding folder). This may reveal issues with the version of the python dependencies, which must be addressed, e.g., with
```sh
pip install "numpy<2.0" scipy matplotlib casadi future-fstrings
pip install --upgrade scipy  # possibly unnecessary
cd $ACADOS_SOURCE_DIR/interfaces/acados_template
pip install -e . # to reinstall the acados python interface
```


### PX4-MPC 
The repo contains an example of an MPC controller communicating with the PX4 flight controller over the microrxce dds bridge. To use it, clone the repo with the mpc simulations
```sh
mkdir -p ~/px4_ws/src # likely already existing 
cd ~/px4_ws/src
git clone https://github.com/PX4/px4_msgs.git  # likely already existing from begore, will throw an error that can be ignored
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git 
git clone https://github.com/DISCOWER/px4-mpc.git  # even better, a forked copy of this repo can be used to be able to modify it (in that case use `clone git@github.com:DISCOWER/px4-mpc.git`)
cd ..
colcon build --packages-up-to px4_mpc
colcon build --packages-select px4_mpc --symlink-install
source install/setup.bash
```
If a fork of px4-mpc is used, it is worth setting px4-mpc as a remote to be able to pull changes from the forked repo and merge them in the fork:
```sh
cd /px4_ws/src/px4-mpc
git remote add upstream git@github.com:DISCOWER/px4-mpc.git
```
NOTES:
- the line `colcon build --packages-select px4_mpc --symlink-install` is useful to be able to modify the px4_mpc code and have the changes take effect without having to rebuild the px4_mpc project every time.
- To be able to modify the px4_mpc code and integrate it effortlessly with our own code, the best approach is to fork the px4-mpc repo and clone that one during the installation steps above.
- In the PX4-MPC folder there is a python file called `mpc_spacecraft.py` which containes all the relevant methods for interfacing with ROS (methods to read/write on PX4 topics). The file can be inspected to learn how to read/write on ros topics to control the atmos spacecraft via the PX4 fligth controller.
- When using a controller running in python, we need to publish to a topic called `offboard_control`, where we basically declare what type of control inputs we are providing to the PX4 flight controller.
- If the px4-mpc is used as a low-level controller, and our mpc as a higher-level one, then some modifications of the px4-mpc module may be required to be able to pass a sequence of states (trajectory) as input, instead of a single setpoint.
- As of 05/05/2026, there is a mismatch between the names of the ros2 topics returned by px4 and those used in px4-mpc. To fix it, go in the `px4_mpc/px4_mpc/mpc_spacecraft.py` file and add the following additional subscription after `self.status_sub_v1`:
	```py
	self.status_sub_v4 = self.create_subscription(
        VehicleStatus,
        'fmu/out/vehicle_status_v4',
        self.vehicle_status_callback,
        qos_profile_sub)
	```


### OpenMPC (optional)
To use the offset-free mode of the PX4-MPC controller, install OpenMPC:
```sh
sudo apt install -y libcdd-dev libgmp-dev python3-dev
git clone git@github.com:mikaelj-kth-se/OpenMPC.git
cd OpenMPC
pip install .
```
With OpenMPC the mosek solver is also required. To set up the license, go on the official mosek website, apply for one, and then place it in the correct folder with:
```sh
mkdir -p ~/mosek
cp </path/to/license>/mosek.lic ~/mosek/mosek.lic
```



-----------------------------------------------------------------------------
## Python Launch Scripts Setup

The repo https://github.com/DISCOWER/px4_launch contains a list of python scripts to launch different types of simulations, including a mult-agent one. The python launch script replaces the `make px4_sitl_spacecraft gz_atmos` and `ros2 launch ...` commands. Using these scripts is the recommended way to run a multi-spacecraft simulation, which can be achieved by launching the `launch_multi_atmos.py`.

To use the scripts, clone the repo (I am using a personal fork where I have made some minor modifications and fixed some bugs):
```sh
cd ~
git clone git@github.com:gbattocletti/px4-launch.git
```
Change the execution permissions of the launch scripts:
```sh
cd px4-launch/
chmod +x launch_*.py
```
Then add the following environment variable to `.bashrc`:
```sh
export PX4_Autopilot_Dir=~/PX4-Autopilot
```
NOTE: to be able to set multiple atmos in offboard mode in QGC, add
```
param set-default COM_RC_IN_MODE 1
```
to the gazebo atmos airframe in PX4, which is located under `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/70000_gz_atmos` (in the future a PR to the PX4 repo will probably be created and this step will not be required anymore)



------------------------------------------------------------------------------
## Simulation (Software in the loop)

To launch a simulation, launch individually the components of the simulation stack in different terminals:

1. Launch Micro-XRCE-DDS-Agent to access PX4 topics from ROS2 and vice versa (in general, to enable the communication between ROS2 and PX4)
	```sh
	microxrce udp4 -p 8888
	```

2. Launch the gazebo simulation. This can be achieved in 2 ways, either by launching the simulation manually (suitable only for single agent simulations) or via python launcher scripts (suitable both for single and multi agent sim). In the first case, navigate to the `PX4-Autopilot` folder and run:
	```sh
	make px4_sitl_spacecraft gz_atmos
	```
	This command simulates both the PX4 controller, which receives control commands from the `in/` ROS topics via microxrce and sends back simulated sensor readings on the `out/` topics, and the dynamics of the spacecraft, which is simulated in Gazebo. Other options that can be used when launching a simulation are:
	```sh
	HEADLESS=1  # no gazebo GUI (no visualization)
	PX4_SIM_SPEED_FACTOR=1  # simulation speedup (only when headless=1?)
	PX4_UXRCE_DDS_NS=pop  # namespace for ros topics
	PX4_GZ_WORLD=kthspacelab  # kth workspace instead of infinite world
	```
	To use the python scripts instead do:
	```sh
	cd px4-launch/
	./launch_single_atmos.py  # or launch_multi_atmos.py
	```
	NOTE: When launching gazebo headles, it is still possible to visualize a running simulation with
	```sh
	gz sim -g
	```
	To kill an already running gazebo process (if the gui was closed without stopping the simulation) run
	```sh
	pkill -9 -f "ign gazebo"
	pkill -f gz
	pkill -f gzserver
	```
	NOTE that, as mentioned, the spacecraft information that can be accessed from the ros topics corresponds to the simulated PX4 sensor readings, and is not the 'true' spacecraft state. The real spacecraft states can instead be accessed via:
	```sh
	gz topic -l
	# Output:
	# /world/<world_name>/pose/info
	# /world/<world_name>/dynamic_pose/info
	# /model/<model_name>/pose
	gz topic -e -t /world/<world_name>/dynamic_pose/info  # echo topic 
	```

3. Start PX4-MPC (or another flight controller if a different one is used). On another terminal, launch
	```sh
	ros2 launch px4_mpc mpc_spacecraft_launch.py 
	```
	Additional arguments are:
	```sh
	mode:=wrench  # other options are available in px4-mpc
	setpoint_from_rviz:=True  # true for manual setpoints, false for my controller 
	namespace:=pop  # same name used in PX4_UXRCE_DDS_NS when launching px4
	```

4. Launch QGroundControl (via `qgroundcontrol` from any location), and run
	```sh
	commander arm
	commander mode offboard
	```
	This can be also achieved manually in the GUI.

5. (optional) List the ROS topics and read their content. In a new terminal:
	```sh
	ros2 topic list
	```
	A specific topic can be listened to with
	```sh
	ros2 topic echo /fmu/out/vehicle_attitude
	```

### Summary: launch simulation
To start a new simulation, launch the following commands in different terminals:
```sh
# launch dds bridge between ros and px4
microxrce udp4 -p 8888

# launch the simulation either manually or via a python launcher script (which simulates both the PX4 flight controller and the spacecraft dynamics in Gazebo) with the full command (after moving to /PX4-Autopilot):
HEADLESS=1 PX4_GZ_WORLD=kthspacelab PX4_UXRCE_DDS_NS=pop make px4_sitl_spacecraft gz_atmos
# or with the python launch script:
./px4-launc/launch_single_atmos.py

# launch the controller (NOTE: for the multi-agent case, multiple nodes need to be launched with different namespaces, each matching a different robot)
ros2 launch px4_mpc mpc_spacecraft_launch.py mode:=wrench setpoint_from_rviz:=True namespace:=pop

# finally, launch qgroundcontrol to arm the robots and have them move
qgroundcontrol
```



------------------------------------------------------------------------------
## Real-world experiments
TODO

------------------------------------------------------------------------------
## .bashrc summary

Summary of all the lines added while installing ros, gazebo, and atmos stuff:
```sh
source /opt/ros/humble/setup.bash
source ~/px4_ws/install/setup.bash
export QT_QPA_PLATFORM=xcb  # fix wsl compositor issues
export GDK_BACKEND=x11  # fix wsl compositor issues
export WAYLAND_DISPLAY=  # fix wsl compositor issues
export PATH=$PATH:/opt/xtensa-esp-elf/bin/
export PX4_Autopilot_Dir=~/PX4-Autopilot
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
alias microxrce="$HOME/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
alias killgz='pkill -9 -f "ign gazebo"; pkill -9 -f "gz sim"; pkill -9 -f gazebo; pkill -9 -f "bin/px4"'
alias killsim='~/px4-launch/launch_single_atmos.py --kill; ~/px4-launch/launch_multi_atmos.py --kill; killgz'
```
Lines 3--5 have been added since on wsl I had some issues with the window compositor, so that the  mouse was going under the windows launched from wsl (e.g., Gazebo). This issue and fix may be very setup-related (depending e.g. on the availability of a discrete GPU)