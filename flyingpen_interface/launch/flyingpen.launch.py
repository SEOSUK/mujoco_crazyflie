from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------- params.yaml ----------
    pkg_share = get_package_share_directory("flyingpen_interface")
    params = os.path.join(pkg_share, "config", "parameters.yaml")
    rviz_config = os.path.join(pkg_share, "config", "flyingpen.rviz")

    # ---------- robot_description (URDF) ----------
    plant_share = get_package_share_directory("plant")
    urdf_path = os.path.join(plant_share, "data", "cf_BLDC.urdf")
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

    # ---------- plant ----------
    # MJ처럼 python module로 실행 (현재 venv/환경 그대로 사용)
    plant_node = ExecuteProcess(
        cmd=[
            "python3", "-m", "plant.plant",
            "--ros-args",
            "-r", "__node:=plant",
            "--params-file", params,
        ],
        output="screen",
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
        parameters=[params],   # ✅ MJ처럼 params 적용
    )

    # ---------- wrench_observer ----------
    wrench_observer_node = Node(
        package="flyingpen",
        executable="wrench_observer",
        name="wrench_observer",
        output="screen",
        parameters=[params],   # ✅ MJ처럼 params 적용
    )

    # ---------- rviz_visual ----------
    rviz_visual_node = Node(
        package="flyingpen_interface",
        executable="rviz_visual",
        name="rviz_visual",
        output="screen",
        parameters=[params],
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

    return LaunchDescription([
        robot_state_publisher_node,
        world_to_wall_tf_node,
        plant_node,
        controller_node,
        trajectory_generation_node,
        wrench_observer_node,
        rviz_visual_node,
        data_logger_node,
        rviz2_node,
    ])

