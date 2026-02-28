#!/usr/bin/env python3
"""ROS2 launch file for simulation mode.

On Windows, Gazebo Classic (gzserver/gzclient) and Gazebo Ignition both
crash at startup due to DLL compatibility issues (STATUS_ENTRYPOINT_NOT_FOUND)
under x64 emulation on ARM64.

Instead, this launch file runs a minimal Python 2D simulator (sim_node.py)
that publishes /odom, /scan, /joint_states, and TF odom→base_link using only
packages already present in ros_env.  The robot moves in RViz when
movement.py sends /cmd_vel commands.
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    urdf_file = os.path.join(_PKG_DIR, 'mini_mec_robot_gazebo.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Resolve package://movement_pkg/ → absolute file URI (platform-safe)
    meshes_uri = Path(_PKG_DIR).as_uri() + '/'
    robot_description = robot_description.replace('package://movement_pkg/', meshes_uri)

    # ── Python 2D simulator ──────────────────────────────────────────────────
    # Publishes /odom, /scan, /joint_states, TF odom→base_link.
    # Uses sys.executable so it runs in the same conda Python as ros2 launch.
    sim_node = ExecuteProcess(
        cmd=[sys.executable,
             os.path.join(_PKG_DIR, 'movement_pkg', 'sim_node.py')],
        output='screen',
    )

    # ── robot_state_publisher ────────────────────────────────────────────────
    # Reads the URDF and publishes /robot_description and TF for fixed joints
    # (base_link→laser_link, base_link→camera_link, etc.).
    # use_sim_time=False: uses wall clock (no /clock source needed).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ── Static TF: odom → odom_combined ─────────────────────────────────────
    # RViz fixed frame is "odom_combined" (the EKF frame used on the real
    # robot).  In simulation there is no EKF, so publish an identity static
    # transform.  sim_node provides the dynamic odom→base_link transform.
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'odom_combined'],
        output='screen',
    )

    return LaunchDescription([
        sim_node,
        robot_state_publisher,
        static_odom_tf,
    ])
