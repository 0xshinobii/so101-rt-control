"""
Bring the SO-101 up in Gazebo Harmonic with ros2_control active.

Deliberately no MoveIt. This file's only job is to get the robot into the
simulator with its controllers running. If `ros2 control list_controllers`
shows joint_state_broadcaster, arm_controller and gripper_controller all
active, the simulation half is correct, and anything that fails afterwards is
MoveIt wiring rather than Gazebo.

Note what is NOT started here: controller_manager. The gz_ros2_control system
plugin declared in the URDF starts one inside the Gazebo process so its update
loop is driven by simulation time. A second, separately launched
controller_manager would fight it for the same hardware interfaces.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_share = get_package_share_directory("so101_moveit_config")
    description_share = get_package_share_directory("so101_description")

    # sdformat resolves package:// URIs by searching this path. Without it the
    # robot spawns as an invisible, collision-less skeleton because every mesh
    # reference fails silently. Append, don't replace: gz_sim.launch.py reads
    # this var to build Gazebo's world/model search path.
    resource_path = os.path.dirname(description_share)

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                "xacro ",
                os.path.join(moveit_share, "config", "so101.urdf.xacro"),
                " sim:=true",
            ]),
            value_type=str,
        )
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
    )

    # Publishes the TF tree and the /robot_description topic that the spawner
    # reads. use_sim_time so every timestamp comes from /clock, not wall clock.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "so101", "-z", "0.0"],
    )

    # Gazebo owns simulation time; this bridge is what makes /clock visible to
    # ROS nodes. Without it every node silently uses wall clock and trajectory
    # timing drifts against the simulation.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
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

    return LaunchDescription([
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description="Gazebo world file to load.",
        ),
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,
        # Controllers are chained off the spawn, not started in parallel: the
        # controller_manager only exists once Gazebo has loaded the model and
        # its gz_ros2_control plugin.
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_controller, gripper_controller],
            )
        ),
    ])
