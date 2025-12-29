# 플라잉펜 말고 무조코 시뮬
## 인스톨
```bash
cd <your workspace>/src
git clone https://github.com/SEOSUK/mujoco_crazyflie --recursive
git checkout just_flight
cd ..
colcon build
```

## 실행 방법
### 노드 실행
```bash
ros2 launch flyingpen_interface flyingpen.launch.py
```
### 위치 명령 전송
```bash
ros2 topic pub --once /crazyflie/in/pos_cmd std_msgs/msg/Float64MultiArray "{data:[x, y, z, yaw]}"
```
여기서 x, y, z, yaw에 각각 목표 위치[m] 및 yaw[rad] 입력


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



