# CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor
This code accompanies the work in the ICRA 2024 accepted paper "CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor" [1]. CrazySim is a simulator platform that runs Crazyflie firmware in a simulation state on a desktop machine with integrated communication with Gazebo sensors and physics engine. The simulated Crazyflie firmware is intended to communicate with a custom Crazyflie Python library ([CFLib](https://github.com/bitcraze/crazyflie-lib-python)) provided in this code. This enables simulating the behavior of CFLib scripts that are intended to control single or multiple Crazyflies in a real hardware demonstration. With CFLib communication capabilities, users can choose to use [CrazySwarm2](https://github.com/IMRCLab/crazyswarm2) with CFLib as the backend for a ROS 2 interface with the simulator. In this code we also provide a case study that uses model predictive control (MPC) using [Acados](https://github.com/acados/acados) for decentralized control of Crazyflie drone fleets.

![](16cfs.gif)

## References

[1] C. Llanes, Z. Kakish, K. Williams, and S. Coogan, “CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor,” To appear in 2024
IEEE International Conference on Robotics and Automation (ICRA), 2024.


```console
@INPROCEEDINGS{LlanesICRA2024,
author = {Llanes, Christian and Kakish, Zahi, and Williams, Kyle and Coogan, Samuel},
booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
title = {CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor},
year = {2024},
pubstate={forthcoming}
}
```

# CrazySim Setup

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

## crazyflie-clients-python [Optional]
[WARNING] This modified client package is only for software-in-the-loop and has several hardware specific features disabled. Do not use this package for your hardware.

If you want to test a single Crazyflie with a custom crazyflie-clients-python for SITL, then run the following command in your terminal. If pip reinstalls cflib, then you may have to remove it and install from source above.

```bash
cd crazyflie-clients-python
pip install -e .
```

https://github.com/gtfactslab/Llanes_ICRA2024/assets/40842920/88fdad50-59a2-4810-bfb2-43c54308ce70


## crazyflie-firmware
[WARNING] This is a modified version of the firmware for software-in-the-loop. At this time do not use this firmware for your hardware. SITL integration with Kbuild is being developed for cross-platform building.

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

Now you can run any CFLib Python script with URI `udp://0.0.0.0:19850`. For drone swarms increment the port for each additional drone.

# Crazyswarm2 and Model Predictive Control Case Study
This section follows the setup of CrazySwarm2 with CrazySim and demonstrating a case study that uses a model predictive controller (MPC) with Acados to track a set of predefined temporally parametrized trajectories.

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

### Start up the Firmware
Start up the firmware with any of the 3 launch script options. Below we demonstrate 4 Crazyflies in a square formation.
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie
```

### Start Crazyswarm2
Make sure that `cf_1`, `cf_2`, `cf_3`, and `cf_4` are enabled in the CrazySwarm2 configuration YAML file. Launch the Crazyswarm2 services with CFLib backend.
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

### Start MPC code
### 
Run the Crazyflie MPC demonstration with the code below. The argument `n_agents` can be modified for the number of agents in your environment.
```bash
ros2 run crazyflie_mpc crazyflie_multiagent_mpc --n_agents=4
```

Using the command line publisher we can command all vehicles to take off using MPC.
```bash
ros2 topic pub -t 50 -r 50 /all/mpc_takeoff std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to start the trajectory.
```bash
ros2 topic pub -t 50 -r 50 /all/mpc_trajectory std_msgs/msg/Empty
```

Using the command line publisher we can command all vehicles to stop the trajectory and hover.
```bash
ros2 topic pub -t 50 -r 50 /all/mpc_hover std_msgs/msg/Empty
```

We also implemented a MPC land feature, but it's still experimental and may result in crashing the drone.
