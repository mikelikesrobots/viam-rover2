#include "viam_rover2/rp2040_sensor_node.hpp"

#include <chrono>
#include <mutex>
#include <utility>

#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float64.hpp"

namespace viam_rover2 {

const size_t kPublishPeriodMs = 10;

Rp2040SensorNode::Rp2040SensorNode(std::shared_ptr<Rp2040Bridge> bridge,
                                   const std::string& controller_namespace,
                                   bool use_sim_time)
    : rclcpp::Node("rp2040_sensor_node",
                   rclcpp::NodeOptions().use_intra_process_comms(true)),
      bridge_(std::move(bridge)) {
  this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time));

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto battery_qos = rclcpp::QoS(10);

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/" + controller_namespace + "/imu", sensor_qos);
  imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw",
                                                               sensor_qos);
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/" + controller_namespace + "/battery", battery_qos);
  battery_state_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", battery_qos);

  auto wheel_qos = rclcpp::QoS(10);
  pub_meas_left_ = this->create_publisher<std_msgs::msg::Float64>(
      "/" + controller_namespace + "/measured_left", wheel_qos);
  pub_meas_right_ = this->create_publisher<std_msgs::msg::Float64>(
      "/" + controller_namespace + "/measured_right", wheel_qos);
  pub_cmd_left_ = this->create_publisher<std_msgs::msg::Float64>(
      "/" + controller_namespace + "/commanded_left", wheel_qos);
  pub_cmd_right_ = this->create_publisher<std_msgs::msg::Float64>(
      "/" + controller_namespace + "/commanded_right", wheel_qos);

  poll_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(kPublishPeriodMs),
      std::bind(&Rp2040SensorNode::poll_and_publish, this));
}

void Rp2040SensorNode::updateMeasuredVelocities(double left, double right) {
  std::lock_guard<std::mutex> lk(wheel_mutex_);
  wheel_meas_left_ = left;
  wheel_meas_right_ = right;
}

void Rp2040SensorNode::updateCommandedVelocities(double left, double right) {
  std::lock_guard<std::mutex> lk(wheel_mutex_);
  wheel_cmd_left_ = left;
  wheel_cmd_right_ = right;
}

void Rp2040SensorNode::poll_and_publish() {
  if (!bridge_) {
    return;
  }

  ImuData imu;
  std::chrono::steady_clock::time_point imu_ts;
  if (bridge_->getLatestImuSnapshot(imu, imu_ts)) {
    if (!have_imu_seq_ || imu.seq != last_imu_seq_) {
      publish_imu(imu);
      last_imu_seq_ = imu.seq;
      have_imu_seq_ = true;
    }
  }

  BatteryData battery;
  std::chrono::steady_clock::time_point battery_ts;
  if (bridge_->getLatestBatterySnapshot(battery, battery_ts)) {
    if (!have_battery_seq_ || battery.seq != last_battery_seq_) {
      publish_battery(battery);
      last_battery_seq_ = battery.seq;
      have_battery_seq_ = true;
    }
  }

  {
    std_msgs::msg::Float64 m;
    std::lock_guard<std::mutex> lk(wheel_mutex_);
    m.data = wheel_meas_left_;
    pub_meas_left_->publish(m);
    m.data = wheel_meas_right_;
    pub_meas_right_->publish(m);
    m.data = wheel_cmd_left_;
    pub_cmd_left_->publish(m);
    m.data = wheel_cmd_right_;
    pub_cmd_right_->publish(m);
  }
}

void Rp2040SensorNode::publish_imu(const ImuData& imu) {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "imu_link";
  msg.linear_acceleration.x = imu.ax;
  msg.linear_acceleration.y = imu.ay;
  msg.linear_acceleration.z = imu.az;
  msg.angular_velocity.x = imu.gx;
  msg.angular_velocity.y = imu.gy;
  msg.angular_velocity.z = imu.gz;
  msg.orientation_covariance[0] = -1.0;
  msg.angular_velocity_covariance[0] = 0.02;
  msg.angular_velocity_covariance[4] = 0.02;
  msg.angular_velocity_covariance[8] = 0.02;
  msg.linear_acceleration_covariance[0] = 0.04;
  msg.linear_acceleration_covariance[4] = 0.04;
  msg.linear_acceleration_covariance[8] = 0.04;
  imu_pub_->publish(msg);
  imu_raw_pub_->publish(msg);
}

void Rp2040SensorNode::publish_battery(const BatteryData& battery) {
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.voltage = static_cast<double>(battery.voltage_mV) / 1000.0;
  msg.current = static_cast<double>(battery.current_mA) / 1000.0;
  msg.percentage = battery.soc;
  battery_pub_->publish(msg);
  battery_state_pub_->publish(msg);
}

}  // namespace viam_rover2