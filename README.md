# 플라잉펜 말고 크플-무조코 시뮬레이션만 있는 버전
crazyflie firmware-like Cascade PID가 구현되어 있음.
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

## For Flying Pen (Ubuntu 24.04)
Python 패키지 충돌을 방지하기 위해 MuJoCo는 ROS 2와 분리된 Python 가상환경에서 실행합니다.

1. Python 가상환경 준비 (venv 생성)
```bash
sudo apt update
sudo apt install -y python3-full python3-venv
```

2. 가상환경 생성
```bash
python3 -m venv ~/venvs/mujoco
```

3. MuJuCo 가상환경 활성화
```bash
source ~/venvs/mujoco/bin/activate
```

4. ROS 2 워크스페이스 환경 설정
```bash
source ~/<your workspace>/install/setup.bash
```

5. Flying Pen 노드 실행
```bash
ros2 launch flyingpen_interface flyingpen.launch.py
```

6. 키보드 입력 노드 실행
```bash
ros2 run flyingpen_interface command_publisher
```