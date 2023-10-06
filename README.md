# CrazySim: A Software-in-the-Loop Simulator for Nano Quadcopter Fleets
This code accompanies the work in the ICRA 2024 submission "CrazySim: A Software-in-the-Loop Simulator for Nano Quadcopter Fleets".

## crazyflie-lib-python
```bash
cd crazyflie-lib-python
pip install -e .
```

## crazyflie-firmware
The installation instructions and usage are referenced in the [documentation](https://github.com/llanesc/crazyflie-firmware/blob/sitl/documentation.md) file.

#### Building the code
First install Gazebo Garden from https://gazebosim.org/docs/garden/install_ubuntu

Run the command to build the firmware and Gazebo plugins.
```bash
cd crazyflie-firmware
mkdir -p sitl_make/build && cd $_
cmake ..
make all
```

## ROS2 building
Make sure you have ROS 2 [Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). Then build the ROS 2 workspace.
```bash
cd ros2_ws
colcon build --symlink-install
```

## Configuration
### Crazyswarm2
The crazyswarm2  configuration files can be found in 
```bash
ros2_ws/src/crazyswarm2/crazyflie/config/
```
The crazyflies.yaml describes the robots currently being used. If a robot is not in the simulator or hardware, then it can be disabled by setting the enabled parameter to false. A more detailed description for crazyswarm2 configurations can be found [here](https://imrclab.github.io/crazyswarm2/usage.html).

### MPC
The main code for the MPC script is in
```bash
ros2_ws/crazyflie_mpc/crazyflie_mpc/crazyflie_multiagent_mpc.py
```
The trajectory type can be changed to a horizontal circle, vertical circle, helix, or a lemniscate trajectory by changing the variable "trajectory_type" in the CrazyflieMPC class.

## Usage
### Start up SITL
Open a terminal and run
```bash
cd crazyflie-firmware
```

We can then run the firmware instance and spawn the models with Gazebo using a single launch script.

#### Option 1: Spawning a single crazyflie model with initial position (x = 0, y = 0)
```bash
bash tools/simulators/gz/sitl_singleagent.sh -m crazyflie -x 0 -y 0
```

#### Option 2: Spawning 8 crazyflie models to form a perfect square
```bash
bash tools/simulators/gz/sitl_multiagent_square.sh -n 8 -m crazyflie
```

#### Option 3: Spawning multiple crazyflie models with positions defined in the *agents.txt* file. New vehicles are defined by adding a new line with comma deliminated initial position *x,y*.
```bash
bash tools/simulators/gz/sitl_multiagent_text.sh -m crazyflie
```

### Start Crazyswarm2
Launch the Crazyswarm2 services with CFLib backend.
```bash
ros2 launch crazyflie launch.py backend:=cflib
```

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
