#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""Custom Nav2 launch file with local gentle-recovery BT override."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Generate launch description for custom Nav2 navigation stack."""
    wandering_aaeon_dir = get_package_share_directory("wandering_aaeon_tutorial")
    viam_rover2_dir = get_package_share_directory("viam_rover2")

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    gentle_nav_to_pose_bt_xml = os.path.join(
        viam_rover2_dir, "config", "navigate_to_pose_gentle_recovery_jazzy.xml"
    )
    default_nav_to_pose_bt_xml = gentle_nav_to_pose_bt_xml
    # Keep through-poses behavior tree from tutorial package.
    default_nav_through_poses_bt_xml = os.path.join(
        wandering_aaeon_dir,
        "behavior_trees",
        "navigate_through_poses_w_replanning_and_recovery_jazzy.xml",
    )

    lifecycle_nodes_full = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
    ]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = RewrittenYaml(
        source_file=params_file, root_key=namespace, param_rewrites={}, convert_types=True
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "params_file", description="Full path to the ROS2 parameters file"
            ),
            SetParameter("use_sim_time", use_sim_time),
            GroupAction(
                [
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_smoother",
                        executable="smoother_server",
                        name="smoother_server",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=remappings,
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_planner",
                        executable="planner_server",
                        name="planner_server",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=remappings,
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_behaviors",
                        executable="behavior_server",
                        name="behavior_server",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_bt_navigator",
                        executable="bt_navigator",
                        name="bt_navigator",
                        output="screen",
                        emulate_tty=True,
                        parameters=[
                            configured_params,
                            {
                                "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_xml,
                                "default_nav_through_poses_bt_xml": default_nav_through_poses_bt_xml,
                            },
                        ],
                        remappings=remappings + [("navigate_to_pose", "navigate_to_pose_bt")],
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_waypoint_follower",
                        executable="waypoint_follower",
                        name="waypoint_follower",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=remappings,
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_velocity_smoother",
                        executable="velocity_smoother",
                        name="velocity_smoother",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=remappings
                        + [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel_smooth")],
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_collision_monitor",
                        executable="collision_monitor",
                        name="collision_monitor",
                        output="screen",
                        emulate_tty=True,
                        parameters=[configured_params],
                        remappings=[("cmd_vel_in", "cmd_vel_smooth"), ("cmd_vel_out", "cmd_vel")],
                        ros_arguments=["--log-level", "warn"],
                    ),
                    Node(
                        package="nav2_lifecycle_manager",
                        executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        output="screen",
                        emulate_tty=True,
                        parameters=[{"autostart": True}, {"node_names": lifecycle_nodes_full}],
                        ros_arguments=["--log-level", "warn"],
                    ),
                ]
            ),
        ]
    )
