# Flying Pen MuJoCo Simulation

## Install
```bash
cd <your workspace>/src
git clone https://github.com/SEOSUK/mujoco_crazyflie --recursive
git checkout just_flight
cd ..
colcon build
```

## Patch Notes 2026.05.04
- MuJoCo interface updated
- Graphics improved
- Mouse controls improved
- Propeller rotation graphics added

## How To Run

### Launch MuJoCo
```bash
ros2 launch flyingpen_interface flyingpen.launch.py
```

### MuJoCo Basic Example - Figure Eight
```bash
ros2 run flyingpen trajectory8
```

### Send a Position Command
```bash
ros2 topic pub --once /crazyflie/in/pos_cmd std_msgs/msg/Float64MultiArray "{data:[x, y, z, yaw]}"
```
Enter target position `[m]` and yaw `[rad]` as `x, y, z, yaw`.


# Flying Pen Simulation  
### From Gazebo to MuJoCo

This repository contains a **Flying Pen** simulation that has been migrated  
from **Gazebo** to **MuJoCo**.

The simulation model and dynamics are based on the **crazyflow** framework,  
and adapted to enable more efficient, stable, and scalable simulation  
in the MuJoCo environment.

## Overview
- Original simulator: Gazebo
- Target simulator: MuJoCo
- Platform: Crazyflie-based aerial manipulation
- Model source: `crazyflow`

## Motivation
MuJoCo provides faster simulation, better contact dynamics,  
and improved numerical stability compared to Gazebo,  
making it more suitable for contact-aware aerial manipulation research  
such as the Flying Pen task.

## Credits
- Model and dynamics inspired by: **crazyflow**


