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
    plant_node = ExecuteProcess(
        cmd=[
            "python3", "-m", "plant.plant",
            "--ros-args", "-r", "__node:=plant"
        ],
        output="screen",
    )

    controller_node = Node(
        package="low_level_controller",
        executable="pid_cascade",
        name="low_level_controller",
        output="screen",
        parameters=[params],
    )

        # ---------- trajectory generation ----------
    trajectory_node = Node(
        package="flyingpen",
        executable="trajectory_generation",
        name="trajectory_generation",
        output="screen",
    )

    return LaunchDescription([
        plant_node,
        controller_node,
        trajectory_node,
    ])
