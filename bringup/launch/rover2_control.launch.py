# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This file was last modified by Michael Hart, a.k.a Mike Likes Robots
# (mikelikesrobots@outlook.com), on 2024-03-07.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyS4",
            description="Serial port for RP2040 telemetry and control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud",
            default_value="115200",
            description="Baud rate for RP2040 serial",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ekf_config_file",
            default_value=os.path.join(
                get_package_share_directory("viam_rover2"),
                "config",
                "ekf.yaml",
            ),
            description="Path to the EKF configuration file.",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("viam_rover2"), "urdf", "rover2.urdf.xacro"]),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("viam_rover2"),
            "config",
            "rover2_controllers.yaml",
        ]
    )

    # ROS 2 control node (controller_manager)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers, robot_description, {"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", "warn"],
    )

    # State publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", "warn"],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    pid_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "pid_controller_left_wheel_joint",
            "pid_controller_right_wheel_joint",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            robot_controllers,
        ],
    )
    robot_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rover2_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    delay_pid_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[pid_controllers_spawner],
        )
    )

    delay_robot_base_after_pid_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pid_controllers_spawner,
            on_exit=[robot_base_controller_spawner],
        )
    )

    ekf_config = LaunchConfiguration("ekf_config_file")
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", "warn"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_pid_after_joint_state_broadcaster_spawner,
        delay_robot_base_after_pid_controller_spawner,
        ekf_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
