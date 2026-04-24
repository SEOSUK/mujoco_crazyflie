from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------- params.yaml ----------
    pkg_share = get_package_share_directory("flyingpen_interface")
    params = os.path.join(pkg_share, "config", "parameters.yaml")
    trajectory_params = os.path.join(pkg_share, "config", "trajectory_generation.yaml")
    normal_params = os.path.join(pkg_share, "config", "normal_vector_estimation.yaml")
    rviz_visual_params = os.path.join(pkg_share, "config", "rviz_visual.yaml")
    rviz_config = os.path.join(pkg_share, "config", "flyingpen.rviz")

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
        parameters=[trajectory_params],
    )

    normal_vector_estimation_node = Node(
        package="flyingpen",
        executable="normal_vector_estimation",
        name="normal_vector_estimation",
        output="screen",
        parameters=[params, normal_params],
    )

    normal_vector_debug_node = Node(
        package="flyingpen_plotter",
        executable="normal_vector_debug",  # normal_vector_debug, mob_consistency
        name="normal_vector_debug",
        output="screen",
        parameters=[params, normal_params, rviz_visual_params],
    )

    # ---------- wrench_observer ----------
    wrench_observer_node = Node(
        package="flyingpen",
        executable="wrench_observer",
        name="wrench_observer",
        output="screen",
        parameters=[params],
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
        normal_vector_estimation_node,
        trajectory_generation_node,
        wrench_observer_node,
        rviz_visual_node,
        data_logger_node,
        TimerAction(period=1.5, actions=[normal_vector_debug_node]),
        TimerAction(period=3.0, actions=[mujoco_bridge_node]),
        TimerAction(period=4.5, actions=[wind_joystick_node]),
    ])
