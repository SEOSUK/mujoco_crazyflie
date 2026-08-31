from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml_config(path):
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def coerce_optional_float(value, field_name):
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return float(value)
    raise ValueError(f"{field_name} must be numeric or null, got {type(value).__name__}.")


def coerce_optional_vec2(value, field_name):
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field_name} must be a length-2 list or null.")
    result = []
    for index, element in enumerate(value):
        if not isinstance(element, (int, float)):
            raise ValueError(f"{field_name}[{index}] must be numeric.")
        result.append(float(element))
    return result


def coerce_vec3(value, field_name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field_name} must be a length-3 list.")
    result = []
    for index, element in enumerate(value):
        if not isinstance(element, (int, float)):
            raise ValueError(f"{field_name}[{index}] must be numeric.")
        result.append(float(element))
    return result


def build_control_physical_overrides(su_params_config):
    control_physical_params = (
        su_params_config.get("wrench_observer", {})
        .get("ros__parameters", {})
    )
    mass = coerce_optional_float(
        control_physical_params.get("mass", 0.04338),
        "wrench_observer.ros__parameters.mass",
    )
    com_bias = coerce_vec3(
        control_physical_params.get("com_bias", [0.0, 0.0, 0.0]),
        "wrench_observer.ros__parameters.com_bias",
    )
    inertia_diag = coerce_vec3(
        control_physical_params.get("inertia_diag", [2.3951e-5, 2.3951e-5, 3.2347e-5]),
        "wrench_observer.ros__parameters.inertia_diag",
    )
    end_effector_offset = coerce_vec3(
        control_physical_params.get("end_effector_offset", [0.09, 0.0, 0.085]),
        "wrench_observer.ros__parameters.end_effector_offset",
    )
    thrust_effectiveness_mismatch_term = coerce_optional_float(
        control_physical_params.get("thrust_effectiveness_mismatch_term", 0.0),
        "wrench_observer.ros__parameters.thrust_effectiveness_mismatch_term",
    )
    thrust_effectiveness_eta_time_coeffs = coerce_optional_vec2(
        control_physical_params.get("thrust_effectiveness_eta_time_coeffs"),
        "wrench_observer.ros__parameters.thrust_effectiveness_eta_time_coeffs",
    )

    mujoco_bridge_overrides = {
        "thrust_effectiveness_mismatch_term": thrust_effectiveness_mismatch_term,
    }
    if thrust_effectiveness_eta_time_coeffs is not None:
        mujoco_bridge_overrides["thrust_effectiveness_eta_time_coeffs"] = (
            thrust_effectiveness_eta_time_coeffs
        )

    return {
        "low_level_controller": {
            "mass": mass,
        },
        "mujoco_bridge": mujoco_bridge_overrides,
        "wrench_observer": {
            "mass": mass,
            "com_bias": com_bias,
            "inertia_diag": inertia_diag,
            "end_effector_offset": end_effector_offset,
            "mob.Jxx": inertia_diag[0],
            "mob.Jyy": inertia_diag[1],
            "mob.Jzz": inertia_diag[2],
        },
        "trajectory_generation": {
            "end_effector_offset": end_effector_offset,
        },
        "fk_ik_transform": {
            "end_effector_offset": end_effector_offset,
        },
    }


def build_control_pipeline_observer_overrides(su_params_config):
    control_params = (
        su_params_config.get("control_pipeline", {})
        .get("ros__parameters", {})
    )
    raw_mode = str(control_params.get("wrench_observation_mode", "eta_t")).strip().lower()

    mode_aliases = {
        "k_ep": "k_ep",
        "consistency": "k_ep",
        "momentum_observer_2nd_order_consistency": "k_ep",
        "mob_2nd_tau": "k_ep",
        "eta_t": "eta_t",
        "mob_eta_t": "eta_t",
    }
    canonical_mode = mode_aliases.get(raw_mode, raw_mode)

    observer_modes = {
        "k_ep": {
            "implemented": True,
            "force_observation_source": "k_ep",
            "ee_applied_wrench_topic": "/crazyflie/out/ee_applied_mob_2nd_tau",
        },
        "eta_t": {
            "implemented": True,
            "force_observation_source": "eta_t",
            "ee_applied_wrench_topic": "/crazyflie/out/ee_applied_mob_eta_t",
        },
    }

    if canonical_mode not in observer_modes:
        supported_modes = ", ".join(observer_modes.keys())
        raise ValueError(
            "Unsupported control_pipeline.wrench_observation_mode "
            f"'{raw_mode}'. Supported modes: {supported_modes}."
        )

    selected_mode = observer_modes[canonical_mode]
    if not selected_mode.get("implemented", False):
        raise NotImplementedError(
            "control_pipeline.wrench_observation_mode "
            f"'{canonical_mode}' is reserved but not implemented yet."
        )

    gain_config = control_params.get("wrench_observation_gains", {}) or {}
    if not isinstance(gain_config, dict):
        raise ValueError(
            "control_pipeline.wrench_observation_gains must be a mapping keyed by mode name."
        )
    k_ep_gain_config = gain_config.get("k_ep", {}) or {}
    eta_t_gain_config = gain_config.get("eta_T", gain_config.get("eta_t", {})) or {}
    for mode_name, mode_gain_config in (
        ("k_ep", k_ep_gain_config),
        ("eta_T", eta_t_gain_config),
    ):
        if not isinstance(mode_gain_config, dict):
            raise ValueError(
                "control_pipeline.wrench_observation_gains."
                f"{mode_name} must be a mapping of gain names to numeric values."
            )

    wrench_observer_overrides = {}
    k_ep_ke_gain = coerce_optional_float(
        k_ep_gain_config.get("ke"),
        "control_pipeline.wrench_observation_gains.k_ep.ke",
    )
    if k_ep_ke_gain is not None:
        wrench_observer_overrides["mob.Ke"] = k_ep_ke_gain
        wrench_observer_overrides["mob.Ke_ep"] = k_ep_ke_gain

    eta_t_gamma = coerce_optional_float(
        eta_t_gain_config.get("gamma"),
        "control_pipeline.wrench_observation_gains.eta_T.gamma",
    )
    if eta_t_gamma is not None:
        wrench_observer_overrides["mob.eta_t.gamma"] = eta_t_gamma

    eta_t_rho_eta = coerce_optional_float(
        eta_t_gain_config.get("rho_eta"),
        "control_pipeline.wrench_observation_gains.eta_T.rho_eta",
    )
    if eta_t_rho_eta is not None:
        wrench_observer_overrides["mob.eta_t.rho_eta"] = eta_t_rho_eta

    return {
        "normal_vector_estimation": {
            "force_observation_source": selected_mode["force_observation_source"],
        },
        "trajectory_generation": {
            "ee_applied_wrench_consistency_topic": selected_mode["ee_applied_wrench_topic"],
        },
        "wrench_observer": wrench_observer_overrides,
    }


def generate_launch_description():
    # ---------- params.yaml ----------
    pkg_share = get_package_share_directory("flyingpen_interface")
    params = os.path.join(pkg_share, "config", "parameters.yaml")
    su_params = os.path.join(pkg_share, "config", "su_params.yaml")
    wrench_observer_params = os.path.join(pkg_share, "config", "wrench_observer.yaml")
    trajectory_params = os.path.join(pkg_share, "config", "trajectory_generation.yaml")
    normal_params = os.path.join(pkg_share, "config", "normal_vector_estimation.yaml")
    rviz_visual_params = os.path.join(pkg_share, "config", "rviz_visual.yaml")
    default_rviz_config = os.path.join(pkg_share, "config", "flyingpen.rviz")
    params_config = load_yaml_config(params)
    su_params_config = load_yaml_config(su_params)
    control_physical_overrides = build_control_physical_overrides(su_params_config)
    control_pipeline_observer_overrides = build_control_pipeline_observer_overrides(
        su_params_config
    )

    panel_config = (
        params_config.get("panel", {})
        .get("ros__parameters", {})
    )
    panel_runtime_params = {
        key: panel_config[key]
        for key in ("history_sec", "update_hz", "render_hz")
        if key in panel_config
    }

    # ---------- robot_description (URDF) ----------
    mujoco_bridge_share = get_package_share_directory("mujoco_bridge")
    urdf_path = os.path.join(mujoco_bridge_share, "data", "cf_BLDC.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ---------- mujoco_bridge ----------
    mujoco_bridge_node = Node(
        package="mujoco_bridge",
        executable="mujoco_bridge_node",
        name="plant",
        output="screen",
        parameters=[
            params,
            control_physical_overrides["mujoco_bridge"],
            {
                "viewer.window_x": 900,
                "viewer.window_y": 40,
                "viewer.window_width": 1000,
                "viewer.window_height": 820,
            },
        ],
    )

    # ---------- controller ----------
    controller_node = Node(
        package="low_level_controller",
        executable="pid_cascade",
        name="low_level_controller",
        output="screen",
        parameters=[params, control_physical_overrides["low_level_controller"]],
    )

    # ---------- trajectory generation ----------
    trajectory_generation_node = Node(
        package="flyingpen",
        executable="trajectory_generation",
        name="trajectory_generation",
        output="screen",
        parameters=[
            params,
            trajectory_params,
            su_params,
            control_physical_overrides["trajectory_generation"],
            control_pipeline_observer_overrides["trajectory_generation"],
        ],
    )

    normal_vector_estimation_node = Node(
        package="flyingpen",
        executable="normal_vector_estimation",
        name="normal_vector_estimation",
        output="screen",
        parameters=[
            params,
            normal_params,
            su_params,
            control_pipeline_observer_overrides["normal_vector_estimation"],
        ],
    )

    selected_panel = str(panel_config.get("selected", "")).strip()
    if not selected_panel:
        for legacy_key in ("lvlf", "force"):
            legacy_cfg = panel_config.get(legacy_key, {})
            if legacy_cfg.get("enable", False):
                selected_panel = str(
                    legacy_cfg.get("executable", f"normal_vector_{legacy_key}")
                ).strip()
                break

    panel_actions = []
    if selected_panel:
        panel_node = Node(
            package="flyingpen_plotter",
            executable=selected_panel,
            name=selected_panel,
            output="screen",
            parameters=[params, normal_params, rviz_visual_params, wrench_observer_params, su_params, panel_runtime_params],
        )
        panel_actions.append(
            TimerAction(
                period=float(panel_config.get("delay_sec", 1.5)),
                actions=[panel_node],
            )
        )

    wind_joystick_node = Node(
        package="flyingpen_interface",
        executable="wind_joystick.py",
        name="wind_joystick",
        output="screen",
    )

    # ---------- wrench_observer ----------
    wrench_observer_node = Node(
        package="flyingpen",
        executable="wrench_observer",
        name="wrench_observer",
        output="screen",
        parameters=[
            wrench_observer_params,
            su_params,
            control_physical_overrides["wrench_observer"],
            control_pipeline_observer_overrides["wrench_observer"],
        ],
    )

    # ---------- rviz_visual ----------
    rviz_visual_node = Node(
        package="flyingpen_interface",
        executable="rviz_visual",
        name="rviz_visual",
        output="screen",
        parameters=[params, rviz_visual_params],
    )

    # ---------- data logger ----------
    data_logger_node = Node(
        package="flyingpen_interface",
        executable="data_logger",
        name="data_logger",
        output="screen",
        parameters=[params],
    )

    firmware_bridge_node = Node(
        package="flyingpen_interface",
        executable="firmware_bridge",
        name="firmware_bridge",
        output="screen",
    )

    # ---------- rviz2 ----------
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    # ---------- fk_ik_transform ----------
    fk_ik_transform_node = Node(
        package="flyingpen_interface",
        executable="fk_ik_transform",
        name="fk_ik_transform",
        output="screen",
        parameters=[
            params,
            control_physical_overrides["fk_ik_transform"],
            {"debug_print_hz": 0.0},
        ],
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to the RViz config file loaded at startup.",
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz2_node,
        robot_state_publisher_node,
        controller_node,
        fk_ik_transform_node,
        normal_vector_estimation_node,
        trajectory_generation_node,
        wrench_observer_node,
        rviz_visual_node,
        data_logger_node,
        firmware_bridge_node,
        *panel_actions,
        TimerAction(period=4.0, actions=[wind_joystick_node]),
        TimerAction(period=3.0, actions=[mujoco_bridge_node]),
    ])
