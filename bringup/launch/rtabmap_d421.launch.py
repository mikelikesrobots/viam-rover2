#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""RTAB-Map launch for Intel RealSense D421 (depth + infrared, no RGB).

Topics:
    - /camera/infra1/image_rect_raw
    - /camera/infra1/camera_info
    - /camera/depth/image_rect_raw
    - /camera/depth/camera_info
    - /scan is available from depthimage_to_laserscan
    - /odometry/filtered is available from robot_localization
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mapping = LaunchConfiguration("mapping")

    params_file = os.path.join(
        get_package_share_directory("viam_rover2"),
        "config",
        "rtabmap_d421.yaml",
    )

    icp_odometry_params = [params_file]

    sensor_remappings = [
        ("rgb/image", "/camera/infra1/image_rect_raw"),
        ("rgb/camera_info", "/camera/infra1/camera_info"),
        ("depth/image", "/camera/depth/image_rect_raw"),
        ("depth/camera_info", "/camera/depth/camera_info"),
    ]

    rtabmap_remappings = sensor_remappings + [
        ("odom", "/odometry/filtered"),
    ]

    icp_odometry_remappings = [
        ("scan", "/scan"),
        ("scan_cloud", "/scan_cloud_unused"),
        ("odom", "/icp_odom"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping",
                default_value="true",
                description="Run RTAB-Map in mapping mode (build/extend the map).",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered",
                description="Odometry topic consumed by RTAB-Map.",
            ),
            Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                output="screen",
                parameters=icp_odometry_params,
                remappings=icp_odometry_remappings,
                ros_arguments=["--log-level", "warn"],
            ),
            # SLAM mode: build / extend the map
            Node(
                condition=IfCondition(mapping),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[params_file, {"delete_db_on_start": True}],
                remappings=rtabmap_remappings,
                ros_arguments=["--log-level", "warn"],
            ),
            # Localization mode: freeze the map, localize against existing DB
            Node(
                condition=UnlessCondition(mapping),
                package="rtabmap_slam",
                executable="rtabmap",
                output="screen",
                parameters=[
                    params_file,
                    {"delete_db_on_start": False, "Mem/IncrementalMemory": "false"},
                ],
                remappings=rtabmap_remappings,
                ros_arguments=["--log-level", "warn"],
            ),
        ]
    )
