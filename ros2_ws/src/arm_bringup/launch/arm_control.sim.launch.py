"""Bring up the SO-101 arm control node in simulation mode."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("arm_bringup"), "config", "sim.yaml"
    )
    return LaunchDescription([
        Node(
            package="arm_control",
            executable="arm_control_node",
            name="arm_control_node",
            output="screen",
            parameters=[params, {"worker_counters": True}],
        ),
    ])
