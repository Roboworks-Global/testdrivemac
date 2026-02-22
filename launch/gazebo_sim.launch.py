#!/usr/bin/env python3
"""Launch Gazebo Classic with the Wheeltec robot for simulation testing."""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

# Resolve paths relative to this file's directory (project root is one level up)
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    urdf_file = os.path.join(_PKG_DIR, 'mini_mec_robot_gazebo.urdf')
    world_file = os.path.join(_PKG_DIR, 'worlds', 'simulation.world')
    sdf_tmp        = '/tmp/wheeltec_robot.sdf'
    urdf_resolved  = '/tmp/wheeltec_robot_resolved.urdf'

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Resolve package://movement_pkg/ → file://<abs-path>/ so mesh files are
    # found without a colcon install step (works for both RViz and Gazebo).
    meshes_file_uri = f'file://{_PKG_DIR}/'
    robot_description = robot_description.replace(
        'package://movement_pkg/', meshes_file_uri)

    # Write the resolved URDF to a temp file for gz sdf conversion.
    with open(urdf_resolved, 'w') as f:
        f.write(robot_description)

    # Gazebo server.
    # libgazebo_ros_init.so is loaded as a system plugin via -s to initialise
    # the ROS2 context inside Gazebo.  This is required so that the robot's
    # model plugins (diff-drive, ray sensor) can publish ROS2 topics after
    # spawning.
    #
    # libgazebo_ros_factory.so is intentionally NOT loaded: adding it as a
    # second -s flag triggers a boost::mutex crash on macOS (confirmed).
    # Robot spawning is handled via gz model instead (see spawn_robot below).
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', world_file,
            '-s', 'libgazebo_ros_init.so',
            '--verbose',
        ],
        output='screen',
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
    )

    # Robot state publisher (broadcasts TFs from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Spawn the robot using Gazebo's own transport layer (gz model), which does
    # not require the /spawn_entity ROS2 service.
    #
    # Step 1 — gz sdf converts the URDF to SDF format and writes it to a temp
    #           file (gz model only accepts SDF, not URDF).
    # Step 2 — gz model inserts the SDF model into the running world.
    #
    # Once spawned, the robot's model plugins find the ROS2 context that
    # libgazebo_ros_init.so already established, so /cmd_vel and /scan appear
    # as normal ROS2 topics.
    #
    # The 6-second delay gives gzserver time to finish loading the world before
    # the spawn command is sent.
    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'gz sdf -p {urdf_resolved} > {sdf_tmp} && '
                    f'gz model --spawn-file {sdf_tmp} '
                    f'--model-name wheeltec_robot '
                    f'--pose "0 0 0.08 0 0 0"'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])
