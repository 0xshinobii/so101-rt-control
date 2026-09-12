"""
Drive the physical SO-101 from MoveIt through ros2_control.

No Gazebo: a real ros2_control_node loads so101_hardware/So101System, and
static_virtual_joint_tfs publishes world → base_link (the Gazebo world joint
is not in this URDF). Controllers, move_group, and RViz are the same as
sim_moveit.launch.py, on wall clock.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("so101_moveit_config")

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("device", default_value="/dev/ttyACM0"))
    ld.add_action(DeclareLaunchArgument("baud", default_value="1000000"))
    ld.add_action(
        DeclareLaunchArgument(
            "calib_file",
            default_value=os.path.join(
                os.path.expanduser("~"),
                "robotics/so101-rt-control/so101_follower_calib.json",
            ),
        )
    )
    ld.add_action(DeclareLaunchArgument("torque_limit", default_value="300"))
    ld.add_action(DeclareLaunchArgument("enable_torque", default_value="true"))

    xacro_file = os.path.join(moveit_share, "config", "so101.urdf.xacro")
    xacro_args = {
        "hardware": "real",
        "device": LaunchConfiguration("device"),
        "baud": LaunchConfiguration("baud"),
        "calib_file": LaunchConfiguration("calib_file"),
        "torque_limit": LaunchConfiguration("torque_limit"),
        "enable_torque": LaunchConfiguration("enable_torque"),
    }

    # Jazzy CM only reads /robot_description from the topic. MoveIt's Xacro
    # substitution is fine as a node parameter, but does not make RSP publish.
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                xacro_file,
                " hardware:=real",
                " device:=", LaunchConfiguration("device"),
                " baud:=", LaunchConfiguration("baud"),
                " calib_file:=", LaunchConfiguration("calib_file"),
                " torque_limit:=", LaunchConfiguration("torque_limit"),
                " enable_torque:=", LaunchConfiguration("enable_torque"),
            ]),
            value_type=str,
        )
    }

    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .robot_description(mappings=xacro_args)
        .to_moveit_configs()
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_share, "launch", "static_virtual_joint_tfs.launch.py")
            )
        )
    )

    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[os.path.join(moveit_share, "config", "ros2_controllers.yaml")],
        )
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "60",
            ],
            output="screen",
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    arm_controller = spawner("arm_controller")
    gripper_controller = spawner("gripper_controller")

    ld.add_action(joint_state_broadcaster)
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_controller, gripper_controller],
            )
        )
    )

    ld.add_action(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
    )
    ld.add_action(
        Node(
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
            ],
        )
    )
    return ld
