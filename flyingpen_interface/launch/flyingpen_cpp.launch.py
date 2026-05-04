from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # ---------- params.yaml ----------
    pkg_share = get_package_share_directory("flyingpen_interface")
    params = os.path.join(pkg_share, "config", "parameters.yaml")
    wrench_observer_params = os.path.join(pkg_share, "config", "wrench_observer.yaml")
    trajectory_params = os.path.join(pkg_share, "config", "trajectory_generation.yaml")
    normal_params = os.path.join(pkg_share, "config", "normal_vector_estimation.yaml")
    rviz_visual_params = os.path.join(pkg_share, "config", "rviz_visual.yaml")
    rviz_config = os.path.join(pkg_share, "config", "flyingpen.rviz")
    with open(params, "r") as f:
        params_config = yaml.safe_load(f) or {}

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

    # ---------- wall TF (world -> wall) ----------
    # args: x y z roll pitch yaw parent child
    world_to_wall_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_wall",
        output="screen",
        arguments=["1.5", "0", "0", "1.57", "0", "0", "world", "wall"],
    )

    # ---------- mujoco_bridge ----------
    mujoco_bridge_node = Node(
        package="mujoco_bridge",
        executable="mujoco_bridge_node",
        name="plant",
        output="screen",
        parameters=[
            params,
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
        parameters=[params],
    )

    # ---------- trajectory generation ----------
    trajectory_generation_node = Node(
        package="flyingpen",
        executable="trajectory_generation",
        name="trajectory_generation",
        output="screen",
        parameters=[trajectory_params, params],
    )

    normal_vector_estimation_pure_node = Node(
        package="flyingpen",
        executable="normal_vector_estimation",
        name="normal_vector_estimation_pure",
        output="screen",
        parameters=[params, normal_params, {
            "force_observation_source": "mob_2nd",
            "contact_frame_quat_topic": "/estimated_contact_frame_quat_pure",
            "contact_force_x_topic": "/normal_vector/contact_force_x_pure",
            "normal_debug_metrics_topic": "/normal_vector/debug_metrics_pure",
        }],
    )

    normal_vector_estimation_ke_node = Node(
        package="flyingpen",
        executable="normal_vector_estimation",
        name="normal_vector_estimation_ke",
        output="screen",
        parameters=[params, normal_params, {
            "force_observation_source": "mob_2nd_tau",
            "contact_frame_quat_topic": "/estimated_contact_frame_quat",
            "contact_force_x_topic": "/normal_vector/contact_force_x",
            "normal_debug_metrics_topic": "/normal_vector/debug_metrics",
        }],
    )

    normal_vector_estimation_ke_alt_node = Node(
        package="flyingpen",
        executable="normal_vector_estimation",
        name="normal_vector_estimation_ke_alt",
        output="screen",
        parameters=[params, normal_params, {
            "force_observation_source": "mob_2nd_tau_ke_alt",
            "contact_frame_quat_topic": "/estimated_contact_frame_quat_ke_alt",
            "contact_force_x_topic": "/normal_vector/contact_force_x_ke_alt",
            "normal_debug_metrics_topic": "/normal_vector/debug_metrics_ke_alt",
        }],
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
            parameters=[params, normal_params, rviz_visual_params, wrench_observer_params, panel_runtime_params],
        )
        panel_actions.append(
            TimerAction(
                period=float(panel_config.get("delay_sec", 1.5)),
                actions=[panel_node],
            )
        )

    # ---------- wrench_observer ----------
    wrench_observer_node = Node(
        package="flyingpen",
        executable="wrench_observer",
        name="wrench_observer",
        output="screen",
        parameters=[wrench_observer_params],
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

    # ---------- rviz2 ----------
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    # ---------- fk_ik_transform ----------
    fk_ik_transform_node = Node(
        package="flyingpen_interface",
        executable="fk_ik_transform",
        name="fk_ik_transform",
        output="screen",
        parameters=[params],
    )

    # ---------- wind joystick ----------
    wind_joystick_node = Node(
        package="flyingpen_interface",
        executable="wind_joystick.py",
        name="wind_joystick",
        output="screen",
        arguments=[
            "--window-x", "1480",
            "--window-y", "760",
            "--window-width", "420",
            "--window-height", "520",
        ],
    )

    return LaunchDescription([
        rviz2_node,
        robot_state_publisher_node,
        world_to_wall_tf_node,
        controller_node,
        fk_ik_transform_node,
        normal_vector_estimation_pure_node,
        normal_vector_estimation_ke_node,
        normal_vector_estimation_ke_alt_node,
        trajectory_generation_node,
        wrench_observer_node,
        rviz_visual_node,
        data_logger_node,
        *panel_actions,
        TimerAction(period=3.0, actions=[mujoco_bridge_node]),
        TimerAction(period=4.5, actions=[wind_joystick_node]),
    ])
