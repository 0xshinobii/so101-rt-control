"""
MoveIt planning into the Gazebo simulation.

Includes sim.launch.py (Gazebo, the robot, and its controllers) and adds
move_group plus RViz. Note what changes versus demo.launch.py: no
ros2_control_node and no mock hardware. The controller_manager already exists
inside Gazebo, and moveit_controllers.yaml points move_group at the same
arm_controller action server you just drove by hand.

Every node here runs on simulation time, so trajectory timestamps agree with
the clock the controller is stepping on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("so101_moveit_config")

    # sim:=true so MoveIt's model matches the one Gazebo spawned, world link
    # and all. A mismatch here shows up as TF errors rather than a clear failure.
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .robot_description(mappings={"sim": "true"})
        .to_moveit_configs()
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "sim.launch.py")
        )
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([sim, move_group, rviz])
