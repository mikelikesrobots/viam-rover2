#pragma once

#ifndef viam_rover2__RP2040_SENSOR_NODE_HPP_
#define viam_rover2__RP2040_SENSOR_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "viam_rover2/rp2040_bridge.hpp"

namespace viam_rover2 {

class Rp2040SensorNode : public rclcpp::Node {
 public:
  explicit Rp2040SensorNode(std::shared_ptr<Rp2040Bridge> bridge,
                            const std::string& controller_namespace,
                            bool use_sim_time = false);

  void updateMeasuredVelocities(double left, double right);
  void updateCommandedVelocities(double left, double right);

 private:
  void poll_and_publish();
  void publish_imu(const ImuData& imu);
  void publish_battery(const BatteryData& battery);

  std::shared_ptr<Rp2040Bridge> bridge_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
  uint32_t last_imu_seq_{0};
  uint32_t last_battery_seq_{0};
  bool have_imu_seq_{false};
  bool have_battery_seq_{false};
  std::mutex wheel_mutex_;
  double wheel_meas_left_{0.0};
  double wheel_meas_right_{0.0};
  double wheel_cmd_left_{0.0};
  double wheel_cmd_right_{0.0};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_meas_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_meas_right_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_cmd_right_;
};

}  // namespace viam_rover2

#endif  // viam_rover2__RP2040_SENSOR_NODE_HPP_