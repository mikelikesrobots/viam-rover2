#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2025 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions
# and limitations under the License.

"""Launch file for Wandering Viam Rover 2 bring-up."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Wandering Viam Rover 2 bring-up."""
    nav2_param_file = os.path.join(
        get_package_share_directory("viam_rover2"), "config", "rover2_nav.param.yaml"
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory("viam_rover2"), "launch", "nav2_bringup.launch.py"
    )
    nav2_launch_args = {
        "params_file": nav2_param_file,
        "namespace": "",
        "use_sim_time": "false",
    }
    wandering_start_period = 20.0

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rover2_control = LaunchConfiguration("use_rover2_control")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_rtabmap = LaunchConfiguration("use_rtabmap")
    mapping = LaunchConfiguration("mapping")

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time if true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rover2_control",
            default_value="true",
            description="Launch viam_rover2 ros2_control stack",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Run viam_rover2 control with mock hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_amr_interface",
            default_value="false",
            description="Launch ros2_amr_interface bridge node",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "amr_interface_params",
            default_value="",
            description="Path to ros2_amr_interface params file (used only if use_amr_interface=true)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rtabmap",
            default_value="true",
            description="Launch RTAB-Map with depth+infra inputs (D421 compatible)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mapping",
            default_value="false",
            description="If true, start fresh RTAB-Map mapping and keep wandering disabled.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "params_file",
            default_value=nav2_param_file,
            description="Full path to the ROS2 parameters file to use for all launched nodes",
        )
    )

    nodes = []

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("viam_rover2"),
                    "launch",
                    "rover2_bringup.launch.py",
                )
            ),
            launch_arguments={
                "use_mock_hardware": use_mock_hardware,
                "use_sim_time": use_sim_time,
                "enable_ekf": "true",
            }.items(),
            condition=IfCondition(use_rover2_control),
        )
    )

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("viam_rover2"),
                    "launch",
                    "rtabmap_d421.launch.py",
                )
            ),
            launch_arguments={
                "mapping": mapping,
                "delete_db_on_start": mapping,
            }.items(),
            condition=IfCondition(use_rtabmap),
        )
    )

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments=nav2_launch_args.items(),
        )
    )

    wandering_app = Node(
        package="wandering_app",
        executable="wandering",
        parameters=[params_file],
        output="screen",
        emulate_tty=True,
        remappings=[("navigate_to_pose", "navigate_to_pose_bt")],
        ros_arguments=["--log-level", "warn"],
    )

    # TimerAction that starts the wandering app after a delay.
    nodes.append(
        TimerAction(
            period=wandering_start_period,
            actions=[wandering_app],
            condition=UnlessCondition(mapping),
        )
    )

    return LaunchDescription(declared_arguments + nodes)
