# CrazySim: A Software-in-the-Loop Simulator for Nano Quadcopter Fleets
This code accompanies the work in the ICRA 2024 submission "CrazySim: A Software-in-the-Loop Simulator for Nano Quadcopter Fleets".

![](16cfs.gif)

## Supported Platforms
This simulator is currently only supported on Ubuntu systems with at least 20.04. This is primarily a requirement from Gazebo Sim. The simulator was built, tested, and verified on 22.04 with Gazebo Garden.

To install this repository use the recursive command as shown below for HTTPS:
```bash
git clone https://github.com/gtfactslab/Llanes_ICRA2024.git --recursive
```

## crazyflie-lib-python
```bash
cd crazyflie-lib-python
pip install -e .
```

## crazyflie-firmware
The installation instructions and usage are referenced in the [documentation](https://github.com/llanesc/crazyflie-firmware/blob/sitl/documentation.md) file.

### Dependencies
Install dependencies using the code below.
```bash
pip install Jinja2
```

#### Building the code
First install Gazebo Garden from https://gazebosim.org/docs/garden/install_ubuntu

Run the command to build the firmware and Gazebo plugins.
```bash
cd crazyflie-firmware
mkdir -p sitl_make/build && cd $_
cmake ..
make all
```

## Crazyswarm2 and MPC code
Make sure you have ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). 

Install the following for Crazyswarm2:
```bash
sudo apt install libboost-program-options-dev libusb-1.0-0-dev
pip3 install rowan transforms3d
sudo apt install ros-humble-tf-transformations
```

If you want to run the MPC code then you will need Acados. Acados can be installed by following their [documentation](https://docs.acados.org/installation/index.html).

Then build the ROS 2 workspace.
```bash
cd ros2_ws
colcon build --symlink-install
```

### Configuration
The crazyswarm2  configuration files can be found in 
```bash
ros2_ws/src/crazyswarm2/crazyflie/config/
```
The crazyflies.yaml describes the robots currently being used. If a robot is not in the simulator or hardware, then it can be disabled by setting the enabled parameter to false. A more detailed description for crazyswarm2 configurations can be found [here](https://imrclab.github.io/crazyswarm2/usage.html).

The main code for the MPC script is in the following:
```bash
ros2_ws/crazyflie_mpc/crazyflie_mpc/crazyflie_multiagent_mpc.py
```
The trajectory type can be changed to a horizontal circle, vertical circle, helix, or a lemniscate trajectory by changing the variable "trajectory_type" in the CrazyflieMPC class.

## Usage
Currently, users have to restart Gazebo after each CFLib connect and disconnect cycle. Supporting a restart cycle without restarting Gazebo is on the list of things to do.

### Start up SITL
Open a terminal and run
```bash
cd crazyflie-firmware
```

We can then run the firmware instance and spawn the models with Gazebo using a single launch script.

#### Option 1: Spawning a single crazyflie model with initial position (x = 0, y = 0)
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -m crazyflie -x 0 -y 0
```

#### Option 2: Spawning 8 crazyflie models to form a perfect square
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 8 -m crazyflie
```

#### Option 3: Spawning multiple crazyflie models with positions defined in the *agents.txt* file. New vehicles are defined by adding a new line with comma deliminated initial position *x,y*.
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_text.sh -m crazyflie
```

### Start Crazyswarm2
Launch the Crazyswarm2 services with CFLib backend.
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

### Start MPC code
### 
Launch the Crazyflie MPC demonstration.
```bash
ros2 launch crazyflie_mpc crazyflie_multiagent_mpc_launch.py
```

Using the command line publisher we can command all vehicles to take off using MPC.
```bash
ros2 topic pub -t 50 -r 50 /all/mpc_takeoff std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to start the trajectory.
```bash
ros2 topic pub -t 50 -r 50 /all/mpc_trajectory std_msgs/msg/Empty
```
