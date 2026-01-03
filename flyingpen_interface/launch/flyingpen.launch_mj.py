from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------- params.yaml ----------
    pkg_share = get_package_share_directory("flyingpen_interface")
    params = os.path.join(pkg_share, "config", "parameters.yaml")

    # ---------- plant: run as python module (uses current env/venv) ----------
    # NOTE: ExecuteProcess에는 parameters=가 없으므로, --params-file로 주입해야 함
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
        parameters=[params],  # 필요 없으면 제거 가능 (안전하게 붙여둠)
    )

    # ---------- data logger ----------
    data_logger_node = Node(
        package="flyingpen_interface",
        executable="data_logger",
        name="data_logger",
        output="screen",
        parameters=[params],  # 필요 없으면 제거 가능 (안전하게 붙여둠)
    )

    return LaunchDescription([
        plant_node,
        controller_node,
        trajectory_generation_node,
        data_logger_node,
    ])
