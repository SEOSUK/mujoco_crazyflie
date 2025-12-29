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

