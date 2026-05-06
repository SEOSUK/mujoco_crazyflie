# Legacy Version

MuJoCo-based Flying Pen legacy branch.

This branch contains the legacy Flying Pen simulation stack, RViz visualization, keyboard teleop, and contact-aware normal estimation experiments built around the second-order MOB pipeline.

## Included Normal Estimation Methods

`normal_estimator_method` supports only the following three modes in [normal_vector_estimation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/normal_vector_estimation.yaml:4):

- `direction`
- `normal_force_based`
- `None`

### Method Summary

- `direction`
  Normalizes the measured force direction and uses it directly as the contact normal.
- `normal_force_based`
  Removes the velocity direction component from the force direction and estimates the normal from the corrected force. This is the latest version.
- `None`
  Disables normal estimation.

## Build

```bash
cd ~/mujoco_crazyflie
colcon build
source install/setup.bash
```

If you are using a dedicated Python environment for MuJoCo, activate it before launching.

## Launch

Main simulation launch:

```bash
cd ~/mujoco_crazyflie
source install/setup.bash
ros2 launch flyingpen_interface flyingpen_cpp.launch.py
```

This launch starts the main Flying Pen stack including MuJoCo, controller, trajectory generation, normal estimation, RViz, and logger nodes.

## Observer Topics

`wrench_observer` publishes the following main topics:

- `/crazyflie/out/mob_2nd`
  Pure second-order MOB wrench in the world frame.
- `/crazyflie/out/mob_2nd_tau`
  Consistency-corrected second-order MOB wrench in the world frame.
- `/crazyflie/out/mob_2nd_tau_base`
  The base second-order force estimate before the consistency residual term is added.
- `/crazyflie/out/mob_2nd_tau_terms`
  Debug topic.
  `force = Kf (p - p_hat_base)` and `torque = Ke [r]_x^T e_tau`.
- `/crazyflie/out/mob_2nd_tau_consistency`
  Debug topic.
  `force = tau_hat_ext_world` and `torque = r x F_hat`.
- `/crazyflie/out/mob_2nd_tau_residual`
  Debug topic.
  `force = e_tau` and `torque.x = rho_tau`.

The corresponding end-effector-applied wrench topics are:

- `/crazyflie/out/ee_applied_mob_2nd`
- `/crazyflie/out/ee_applied_mob_2nd_tau`
- `/crazyflie/out/ee_applied_mob_2nd_tau_base`

Practical meaning:

- `mob_2nd` is the pure second-order observer output.
- `mob_2nd_tau` is the output after adding the consistency residual correction.
- `mob.Ke` in [wrench_observer.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/wrench_observer.yaml:1) is the consistency residual gain.
- `mob.epsilon_tau` is the small denominator protection used when computing the residual normalization term `rho_tau`.

## Normal Topics

Two `normal_vector_estimation` nodes are launched by default:

- `normal_vector_estimation_pure`
  Input source: `/crazyflie/out/ee_applied_mob_2nd`
- `normal_vector_estimation_ke`
  Input source: `/crazyflie/out/ee_applied_mob_2nd_tau`

Published normal-estimation topics:

- Pure observer normal
  `/estimated_contact_frame_quat_pure`
  `/normal_vector/contact_force_x_pure`
  `/normal_vector/debug_metrics_pure`
- Consistency-corrected normal
  `/estimated_contact_frame_quat`
  `/normal_vector/contact_force_x`
  `/normal_vector/debug_metrics`

Meaning of the main normal outputs:

- `/estimated_contact_frame_quat_pure`
  Contact frame estimated from the pure second-order MOB source.
- `/estimated_contact_frame_quat`
  Contact frame estimated from the consistency-corrected MOB source.
- `/normal_vector/debug_metrics`
  Main debug topic for the `normal_force_based` pipeline.
  RViz uses this to visualize `ke_raw_normal` and `ke_projected_normal`.

Current RViz normal markers:

- `/rviz/ke_force_normal_raw`
  Raw force-direction normal.
- `/rviz/ke_force_normal_projected`
  Velocity-projected normal.
- `/rviz/true_contact_normal`
  Ground-truth wall/cylinder normal used for comparison.

## YAML Usage

### `wrench_observer.yaml`

Main tuning parameters in [wrench_observer.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/wrench_observer.yaml:1):

- `mob.Kf_2nd_order`
  Linear force observer gain.
- `mob.Ktau_2nd_order`
  Angular torque observer gain.
- `mob.mob_alpha_2nd_order`
  Output LPF factor for published MOB wrench.
- `mob.Kp`
  Momentum-state correction gain for linear dynamics.
- `mob.KpTau`
  Momentum-state correction gain for angular dynamics.
- `mob.Ke`
  Consistency residual gain.
  This is the main gain that scales the residual term added to the consistency observer.
- `mob.epsilon_tau`
  Residual normalization safeguard.
  Prevents division-by-zero when the torque residual metric is computed.

### `normal_vector_estimation.yaml`

Main parameters in [normal_vector_estimation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/normal_vector_estimation.yaml:1):

- `reference_object`
  Usually `end_effector`.
  This decides whether the normal estimator uses drone kinematics or end-effector kinematics.
- `force_observation_source`
  Selects the wrench source.
  `mob_2nd` uses the pure observer and `mob_2nd_tau` uses the consistency-corrected observer.
- `normal_force_threshold`
  Minimum force magnitude required before a normal update is accepted.
- `normal_lpf_alpha`
  LPF used by the simple `direction` estimator.

Important `normal_force_based` parameters:

- `velocity_epsilon`
  Velocity dead-zone threshold.
  If contact-point speed is smaller than this, the velocity direction is treated as unreliable.
- `force_epsilon`
  Minimum force magnitude required to trust the raw force-direction normal candidate.
- `algebraic_force_epsilon`
  Minimum corrected-force magnitude required to trust the algebraic projected candidate.
- `gamma_epsilon`
  Soft velocity dead-zone gain used in
  `gamma_v = ||v||^2 / (||v||^2 + gamma_epsilon)`.
  Larger values make the projection trust velocity less aggressively near zero speed.
- `output_lpf_cutoff_rad_s`
  Output normal-vector LPF cutoff.
  This affects the smoothed directions used for `ke_raw_normal`, `ke_projected_normal`, and the gamma-projected direction.
- `candidate_lpf_cutoff_hz`
  LPF cutoff for the algebraic corrected-force candidate before the final normal is formed.
- `beta_n`
  Memory forgetting gain for the projected-normal evidence matrix.
- `sigma_n`
  Memory integration gain for the projected-normal evidence matrix.

Topic mapping summary:

- `ke_raw_normal`
  RViz topic: `/rviz/ke_force_normal_raw`
  Meaning: force direction before velocity projection.
- `ke_projected_normal`
  RViz topic: `/rviz/ke_force_normal_projected`
  Meaning: force direction after removing the velocity-direction component.
- Final consistency-based contact frame
  ROS topic: `/estimated_contact_frame_quat`
  Source: `/crazyflie/out/ee_applied_mob_2nd_tau`
- Final pure-observer contact frame
  ROS topic: `/estimated_contact_frame_quat_pure`
  Source: `/crazyflie/out/ee_applied_mob_2nd`

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

## Useful Files

- [parameters.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/parameters.yaml:1)
- [normal_vector_estimation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/normal_vector_estimation.yaml:1)
- [trajectory_generation.yaml](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/trajectory_generation.yaml:1)
- [flyingpen_cpp.launch.py](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/launch/flyingpen_cpp.launch.py:1)
- [flyingpen.rviz](/home/seosuk/mujoco_crazyflie/src/flyingpen_interface/config/flyingpen.rviz:1)

## Notes

- The default normal estimator is `normal_force_based`.
- RViz starts from the saved config in `flyingpen.rviz`.
