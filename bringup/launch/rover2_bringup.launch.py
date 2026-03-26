import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false", description="Start robot with mock hardware"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time", default_value="false", description="Use simulated time"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_vision", default_value="true", description="Launch vision nodes"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_ekf", default_value="true", description="Launch EKF in rover2_control stack"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ekf_config_file",
            default_value=os.path.join(
                get_package_share_directory("viam_rover2"), "config", "ekf.yaml"
            ),
            description="Path to EKF configuration file",
        )
    )

    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_vision = LaunchConfiguration("enable_vision")
    enable_ekf = LaunchConfiguration("enable_ekf")
    ekf_config_file = LaunchConfiguration("ekf_config_file")

    # Import and launch rover2_control
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("viam_rover2"),
                "launch",
                "rover2_control.launch.py",
            )
        ),
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "use_sim_time": use_sim_time,
            "enable_ekf": enable_ekf,
            "ekf_config_file": ekf_config_file,
        }.items(),
    )

    # Static transform: base_link -> camera_link (camera mounted 9 cm forward, 16 cm up)
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        condition=IfCondition(enable_vision),
        arguments=["0.09", "0", "0.16", "0", "0", "0", "base_link", "camera_link"],
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", "warn"],
    )

    # Construct vision nodes: RealSense D421 and node to convert depth image to laser scan
    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        condition=IfCondition(enable_vision),
        output="screen",
        namespace="",
        parameters=[
            {
                "align_depth.enable": True,
                "pointcloud.enable": False,
                "enable_sync": True,
                "enable_infra1": True,
                "use_sim_time": use_sim_time,
            }
        ],
        ros_arguments=["--log-level", "warn"],
    )

    depthimage_to_laserscan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        condition=IfCondition(enable_vision),
        remappings=[
            ("/depth", "/camera/depth/image_rect_raw"),
            ("/depth_camera_info", "/camera/depth/camera_info"),
        ],
        parameters=[{"scan_time": 0.033}, {"range_min": 0.1}, {"range_max": 2.5}],
        ros_arguments=["--log-level", "warn"],
    )

    # Convert /cmd_vel Twist messages to TwistStamped on /rover2_base_controller/cmd_vel
    stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        remappings=[("cmd_vel_in", "/cmd_vel"), ("cmd_vel_out", "/rover2_base_controller/cmd_vel")],
        parameters=[{"use_sim_time": use_sim_time}],
        ros_arguments=["--log-level", "warn"],
    )

    nodes = [
        control_launch,
        camera_tf,
        realsense_node,
        stamper_node,
        depthimage_to_laserscan_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
