from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("flyingpen_interface")
    params = os.path.join(pkg_share, "config", "parameters.yaml")

    plant_node = Node(
        package="plant",
        executable="plant",
        name="plant",
        output="screen",
    )

    controller_node = Node(
        package="low_level_controller",
        executable="pid_cascade",
        name="low_level_controller",   # <- YAML 키와 맞추는 게 베스트
        output="screen",
        parameters=[params],
    )

    return LaunchDescription([plant_node, controller_node])

