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
        parameters=[params],   # âœ… ì¶”ê°€ (plant: ros__parameters: noise: ... ì½ìŒ)
    )

    controller_node = Node(
        package="low_level_controller",
        executable="pid_cascade",
        name="low_level_controller",
        output="screen",
        parameters=[params],
    )

<<<<<<< HEAD
    data_logger_node = Node(
        package="flyingpen_interface",
        executable="data_logger",
        name="data_logger",
        output="screen",
    )

    trajectory_generation_node = Node(
        package="flyingpen",
        executable="trajectory_generation",
        name="trajectory_generation",
        output="screen",
    )

    return LaunchDescription([
        plant_node,
        controller_node,
        trajectory_generation_node,
        data_logger_node,
=======
    return LaunchDescription([
        plant_node,
        controller_node,
>>>>>>> 871f306 (jazzyë²„ì „ì—ì„œ launchë¥¼ ì“°ê¸° ìœ„í•œ ìˆ˜ì •)
    ])

