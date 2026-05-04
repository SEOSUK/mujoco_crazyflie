# Legacy Version

MuJoCo-based Flying Pen legacy branch.

This version contains multiple normal estimation logics and supporting RViz / teleop utilities for the contact-aware Flying Pen simulation.

## Included Normal Estimation Methods

The normal estimator supports the following methods through `normal_estimator_method` in [normal_vector_estimation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/normal_vector_estimation.yaml:4):

- `direct`
- `kroc`
- `action_normal`
- `Lf_Lv_fusion`
- `normal_force_based`
- `None`

### Main Variants

- `Lf_Lv_fusion`
  Combines force evidence and velocity-geometry evidence with consistency weighting.
- `normal_force_based`
  Uses force-dominant constrained normal estimation with velocity-admissibility correction and projector-memory smoothing.
- `direct`
  Direct normal construction from measured force direction.
- `kroc`
  Legacy estimator path kept for comparison.
- `action_normal`
  Action-based legacy estimator path kept for comparison.

## Build

```bash
cd ~/mujoco_crazyflie
colcon build
source install/setup.bash
```

If you are using a separate Python environment for MuJoCo, activate it before launching.

## Launch

Main simulation launch:

```bash
cd ~/mujoco_crazyflie
source install/setup.bash
ros2 launch flyingpen_interface flyingpen_cpp.launch.py
```

This launch brings up the main Flying Pen stack including:

- `mujoco_bridge`
- `low_level_controller`
- `trajectory_generation`
- `normal_vector_estimation`
- `wrench_observer`
- `rviz_visual`
- `rviz2`
- `data_logger`

## Keyboard Teleop

Run the keyboard teleop node in a separate terminal:

```bash
cd ~/mujoco_crazyflie
source install/setup.bash
ros2 run flyingpen_interface command_publisher
```

### Default Key Bindings

- `w / s`: x command
- `a / d`: y command
- `e / q`: z command
- `z / c`: yaw command
- `x`: reset position command
- `i`: toggle position / velocity mode
- `j / k / l`: force command increase / decrease / reset
- `g`: toggle yz trajectory in velocity mode
- `t`: quit teleop

## Useful Config Files

- [parameters.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/parameters.yaml:1)
- [normal_vector_estimation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/normal_vector_estimation.yaml:1)
- [trajectory_generation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/trajectory_generation.yaml:1)
- [flyingpen_cpp.launch.py](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/launch/flyingpen_cpp.launch.py:1)

## Notes

- RViz starts from the saved config in [flyingpen.rviz](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/flyingpen.rviz:1).
- The default normal estimator config currently selects `normal_force_based`.
