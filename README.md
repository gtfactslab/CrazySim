# CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor
This code accompanies the work in the ICRA 2024 paper "CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor" [1]. CrazySim is a simulator platform that runs Crazyflie firmware in a simulation state on a desktop machine with integrated communication with Gazebo sensors and physics engine. The simulated Crazyflie firmware is intended to communicate with a custom Crazyflie Python library ([CFLib](https://github.com/bitcraze/crazyflie-lib-python)) provided in this code. This enables simulating the behavior of CFLib scripts that are intended to control single or multiple Crazyflies in a real hardware demonstration. With CFLib communication capabilities, users can choose to use [CrazySwarm2](https://github.com/IMRCLab/crazyswarm2) with CFLib as the backend for a ROS 2 interface with the simulator. 

![Architecture Diagram](https://github.com/user-attachments/assets/94f180aa-f7e7-42e8-b877-ce350958b0f1)

## References

[1] C. Llanes, Z. Kakish, K. Williams, and S. Coogan, “CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor,” 
2024 IEEE International Conference on Robotics and Automation (ICRA), 2024.


```console
@INPROCEEDINGS{LlanesICRA2024,
  author={Llanes, Christian and Kakish, Zahi and Williams, Kyle and Coogan, Samuel},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={CrazySim: A Software-in-the-Loop Simulator for the Crazyflie Nano Quadrotor}, 
  year={2024},
  volume={},
  number={},
  pages={12248-12254},
  keywords={Sockets;Prediction algorithms;Hardware;Robustness;Sensors;Trajectory;Task analysis},
  doi={10.1109/ICRA57147.2024.10610906}}

```

# CrazySim Setup

## Supported Platforms
This simulator is currently only supported on Ubuntu systems with at least 20.04. This is primarily a requirement from Gazebo Sim. The simulator was built, tested, and verified on 22.04 with Gazebo Garden.

To install this repository use the recursive command as shown below for HTTPS:
```bash
git clone https://github.com/gtfactslab/CrazySim.git --recursive
```

## crazyflie-lib-python
The official cflib now supports our udpdriver implementation as of [99ad0e3](https://github.com/bitcraze/crazyflie-lib-python/commit/99ad0e3e5be8ec717fd1b0fce0b7320e4acefe6e). Install the official cflib through the [official install instructions](https://github.com/bitcraze/crazyflie-lib-python/blob/master/docs/installation/install.md).

## crazyflie-clients-python [Optional]
If you want to test a single Crazyflie with crazyflie-clients-python for SITL, then run the following commands in your terminal. If pip reinstalls cflib, then you may have to remove it and install from source above.

We have verified success with commit [`d649b66`](https://github.com/bitcraze/crazyflie-clients-python/commit/d649b6615a58ac0eb34aa72a4edef4c5d821eeab).
```bash
git clone https://github.com/bitcraze/crazyflie-clients-python
cd crazyflie-clients-python
pip install -e .
```

## crazyflie-firmware
[WARNING] This is a modified version of the crazyflie-firmware for software-in-the-loop. At this time do not use this firmware for your hardware. SITL integration with Kbuild is being developed for cross-platform building.

The installation instructions and usage are referenced in the [documentation](https://github.com/llanesc/crazyflie-firmware/blob/crazysim/documentation.md) file.

### Dependencies
Run the following commands to install dependencies.
```bash
sudo apt install cmake build-essential
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

## How to use

### Start up SITL
Open a terminal and run
```bash
cd crazyflie-firmware
```

We can then run the firmware instance and spawn the models with Gazebo using a launch script. All launch scripts require a model argument `-m`. All currently implemented models are tabulated below.

| Models | Description |
| --- | --- |
| crazyflie | The default Crazyflie 2.1. |
| crazyflie_thrust_upgrade | The Crazyflie 2.1 with thrust upgrade bundle ([cf2x_T350](https://github.com/utiasDSL/drone-models) parameters). |

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

You can also test a single crazyflie using the custom client if you installed it from the crazyflie-clients-python section.

First start up the custom client.
```bash
cfclient
```

Click on the SITL checkbox, scan, and connect. Once it's connected you can take off and fly using the command based flight controls.

### PID Tuning Example
One use case for simulating a crazyflie with the client is real time PID tuning. If you created a custom crazyflie with larger batteries, multiple decks, and upgraded motors, then it would be useful to tune the PIDs in a simulator platform before tuning live on hardware. An example of real time PID tuning is shown below.

https://github.com/gtfactslab/Llanes_ICRA2024/assets/40842920/b865127c-1b0d-4f49-941d-e57aecda9a54




# Crazyswarm2

This section follows the setup of Crazyswarm2 with CrazySim. We provide an example workflow of launching 4 Crazyflies using CrazySim and connect them to Crazyswarm2.

1. Make sure you have ROS 2 [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. Build the Crazyswarm2 workspace provided as a submodule.
```bash
cd crazyswarm2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Configuration
The crazyswarm2 configuration files can be found in
```bash
crazyswarm2_ws/src/crazyswarm2/crazyflie/config/
```
The crazyflies.yaml describes the robots currently being used. If a robot is not in the simulator or hardware, then it can be disabled by setting the enabled parameter to false. A more detailed description for crazyswarm2 configurations can be found [here](https://imrclab.github.io/crazyswarm2/usage.html).

For the following demo make the following adjustments to the ***robots*** and ***robot_types*** in crazyflies.yaml.

```YAML
robots:
  cf_1:
      enabled: true
      uri: udp://0.0.0.0:19850
      initial_position: [0.0, 0.0, 0.0]
      type: cf_sim

  cf_2:
    enabled: true
    uri: udp://0.0.0.0:19851
    initial_position: [1.0, 0.0, 0.0]
    type: cf_sim

  cf_3:
    enabled: true
    uri: udp://0.0.0.0:19852
    initial_position: [0.0, 1.0, 0.0]
    type: cf_sim 

  cf_4:
    enabled: true
    uri: udp://0.0.0.0:19853
    initial_position: [1.0, 1.0, 0.0]
    type: cf_sim

robot_types:
  cf_sim:
    motion_capture:
      tracking: "vendor"
    big_quad: false
    firmware_logging:
      enabled: true
      default_topics:
        pose:
          frequency: 10
```

### Start up the Firmware
Start up the firmware with any of the 3 launch script options. Below we demonstrate 4 Crazyflies in a square formation.
```bash
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh -n 4 -m crazyflie
```

### Start Crazyswarm2
Make sure that `cf_1`, `cf_2`, `cf_3`, and `cf_4` are enabled in the CrazySwarm2 configuration YAML file. Launch the Crazyswarm2 services with the CFLib backend:
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

Or with the C++ backend:
```bash
ros2 launch crazyflie launch.py backend:=cpp
```

## Model Predictive Control example

The model predictive control example from [1] has been moved to a separate [repository](https://github.com/llanesc/crazyflie-mpc-example).

## Versions
| Version | Description |
| --- | --- |
| 1.0 | Initial release |
| 1.1 | Added receiver thread for CFLib UdpDriver, new thrust upgrade model to Gazebo, and a seperate MPC solver thread with a queue for storing the controls. |
| 1.2 | Merge crazyflie-firmware with commits up to [dbb09b5](https://github.com/bitcraze/crazyflie-firmware/commit/dbb09b5ca16f0ddf63e98d2c44d247a3aa15f056), update submodule motion_capture_tracking to version 1.0.5, fixed Gazebo sending external pose to firmware (wasn't receiving orientation), cleaned up launch scripts, removed some firmware module copies for sitl. |
| 1.3 | Rewritten CFLib UDP driver with threaded receiver and scan_interface for auto-discovery on ports 19850-19859, activity-based connection detection in Gazebo plugin, added SITL deck and battery parameter stubs for cfclient compatibility, updated thrust upgrade model with cf2x_T350 parameters, added Crazyswarm2 as a submodule, added UDP support for the Crazyswarm2 C++ backend, added Crazyswarm2 attitude setpoints. |
