#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ForwardVelocityProbe(Node):
    def __init__(self, speed_mps, target_distance_m, settle_time_s):
        super().__init__("forward_velocity_probe")

        self.speed_mps = float(speed_mps)
        self.target_distance_m = float(target_distance_m)
        self.settle_time_s = float(settle_time_s)
        self.command_duration_s = self.target_distance_m / self.speed_mps

        self.cmd_pub = self.create_publisher(TwistStamped, "/rover2_base_controller/cmd_vel", 10)
        self.create_subscription(Imu, "/rover2_base_controller/imu", self.on_imu, 100)
        self.create_subscription(Odometry, "/rover2_base_controller/odom", self.on_odom, 100)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_ekf, 100)

        self.latest_odom_vx = None
        self.latest_ekf_vx = None

        self.imu_bias_samples = []
        self.imu_bias_ax = 0.0

        self.imu_last_stamp = None
        self.odom_last_stamp = None
        self.ekf_last_stamp = None

        self.imu_velocity = 0.0
        self.imu_distance = 0.0
        self.odom_distance = 0.0
        self.ekf_distance = 0.0

        self.imu_v_samples = []
        self.odom_v_samples = []
        self.ekf_v_samples = []

        self.phase = "settle"
        self.start_monotonic = time.monotonic()
        self.command_start_monotonic = self.start_monotonic + self.settle_time_s
        self.command_stop_monotonic = self.command_start_monotonic + self.command_duration_s
        self.command_actual_start = None
        self.command_actual_stop = None
        self.finished = False

        self.timer = self.create_timer(0.02, self.on_timer)

    @staticmethod
    def stamp_to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def publish_cmd(self, linear_x):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def in_command_window(self):
        return self.phase == "forward"

    def on_imu(self, msg):
        stamp = self.stamp_to_sec(msg.header.stamp)
        ax = float(msg.linear_acceleration.x)

        if self.phase == "settle":
            self.imu_bias_samples.append(ax)
            return

        if self.phase != "forward":
            return

        if self.imu_last_stamp is None:
            self.imu_last_stamp = stamp
            return

        dt = stamp - self.imu_last_stamp
        self.imu_last_stamp = stamp
        if dt <= 0.0 or dt > 0.2:
            return

        # Integrate accel -> velocity -> distance in forward axis.
        accel_forward = ax - self.imu_bias_ax
        self.imu_velocity += accel_forward * dt
        self.imu_distance += self.imu_velocity * dt
        self.imu_v_samples.append(self.imu_velocity)

    def on_odom(self, msg):
        stamp = self.stamp_to_sec(msg.header.stamp)
        self.latest_odom_vx = float(msg.twist.twist.linear.x)

        if self.phase != "forward":
            return

        if self.odom_last_stamp is None:
            self.odom_last_stamp = stamp
            return

        dt = stamp - self.odom_last_stamp
        self.odom_last_stamp = stamp
        if dt <= 0.0 or dt > 0.2:
            return

        self.odom_distance += self.latest_odom_vx * dt
        self.odom_v_samples.append(self.latest_odom_vx)

    def on_ekf(self, msg):
        stamp = self.stamp_to_sec(msg.header.stamp)
        self.latest_ekf_vx = float(msg.twist.twist.linear.x)

        if self.phase != "forward":
            return

        if self.ekf_last_stamp is None:
            self.ekf_last_stamp = stamp
            return

        dt = stamp - self.ekf_last_stamp
        self.ekf_last_stamp = stamp
        if dt <= 0.0 or dt > 0.2:
            return

        self.ekf_distance += self.latest_ekf_vx * dt
        self.ekf_v_samples.append(self.latest_ekf_vx)

    @staticmethod
    def safe_mean(values):
        if not values:
            return float("nan")
        return sum(values) / float(len(values))

    def on_timer(self):
        now = time.monotonic()

        if now < self.command_start_monotonic:
            self.phase = "settle"
            self.publish_cmd(0.0)
            return

        if now < self.command_stop_monotonic:
            if self.phase != "forward":
                self.phase = "forward"
                self.command_actual_start = time.monotonic()
                if self.imu_bias_samples:
                    self.imu_bias_ax = sum(self.imu_bias_samples) / float(
                        len(self.imu_bias_samples)
                    )
                self.get_logger().info(
                    f"Starting forward command: vx={self.speed_mps:.3f} m/s, "
                    f"duration={self.command_duration_s:.3f} s, bias_ax={self.imu_bias_ax:.6f} m/s^2"
                )
            self.publish_cmd(self.speed_mps)
            return

        if not self.finished:
            self.phase = "stop"
            self.publish_cmd(0.0)
            self.command_actual_stop = time.monotonic()
            self.finished = True
            self.print_summary()
            raise SystemExit(0)

    def print_summary(self):
        cmd_duration_actual = float("nan")
        if self.command_actual_start is not None and self.command_actual_stop is not None:
            cmd_duration_actual = self.command_actual_stop - self.command_actual_start

        expected_distance_nominal = self.speed_mps * self.command_duration_s
        expected_distance_actual_time = (
            self.speed_mps * cmd_duration_actual
            if math.isfinite(cmd_duration_actual)
            else float("nan")
        )

        print("\n--- Forward Velocity Probe Results ---")
        print(f"Commanded speed:                 {self.speed_mps:.4f} m/s")
        print(f"Target distance:                 {self.target_distance_m:.4f} m")
        print(f"Nominal command duration:        {self.command_duration_s:.4f} s")
        print(f"Actual command duration:         {cmd_duration_actual:.4f} s")
        print(f"Expected distance (nominal t):   {expected_distance_nominal:.4f} m")
        print(f"Expected distance (actual t):    {expected_distance_actual_time:.4f} m")
        print()
        print(f"IMU mean forward velocity:       {self.safe_mean(self.imu_v_samples):.4f} m/s")
        print(f"Odom mean forward velocity:      {self.safe_mean(self.odom_v_samples):.4f} m/s")
        print(f"EKF mean forward velocity:       {self.safe_mean(self.ekf_v_samples):.4f} m/s")
        print()
        print(f"IMU integrated distance:         {self.imu_distance:.4f} m")
        print(f"Odom integrated distance:        {self.odom_distance:.4f} m")
        print(f"EKF integrated distance:         {self.ekf_distance:.4f} m")
        print()
        print(f"IMU sample count:                {len(self.imu_v_samples)}")
        print(f"Odom sample count:               {len(self.odom_v_samples)}")
        print(f"EKF sample count:                {len(self.ekf_v_samples)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.10, help="Forward command speed in m/s")
    parser.add_argument(
        "--distance", type=float, default=0.50, help="Target command distance in meters"
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=1.0,
        help="Seconds to collect IMU accel bias at rest",
    )
    args = parser.parse_args()

    if args.speed <= 0.0:
        raise ValueError("--speed must be > 0")
    if args.distance <= 0.0:
        raise ValueError("--distance must be > 0")

    rclpy.init()
    node = ForwardVelocityProbe(args.speed, args.distance, args.settle_time)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        try:
            node.publish_cmd(0.0)
            node.publish_cmd(0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
