// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This file was last modified by Michael Hart, a.k.a Mike Likes Robots
// (mikelikesrobots@outlook.com), on 2026-05-22.

#include "viam_rover2/rover2_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using namespace viam_rover2;

Rover2SystemHardware::~Rover2SystemHardware() { stop_sensor_executor(); }

hardware_interface::CallbackReturn Rover2SystemHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  last_tick_counts_.resize(info_.joints.size(), 0);
  last_direction_.resize(info_.joints.size(), 1);

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("Rover2SystemHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("Rover2SystemHardware"),
                   "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Accept at least a position state interface; velocity state will be
    // provided by this hardware implementation as an additional state.
    bool has_position = false;
    for (const auto& si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_position = true;
    }
    if (!has_position) {
      RCLCPP_FATAL(rclcpp::get_logger("Rover2SystemHardware"),
                   "Joint '%s' does not provide required '%s' state interface.", joint.name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Read per-encoder ticks_per_rev
  auto get_param = [&](const std::string& key, double fallback) {
    return info_.hardware_parameters.count(key) ? std::stod(info_.hardware_parameters[key])
                                                : fallback;
  };
  ticks_per_rev_.push_back(get_param("left_ticks_per_rev", 1.0));
  ticks_per_rev_.push_back(get_param("right_ticks_per_rev", 1.0));
  encoder_telemetry_hz_ = static_cast<uint32_t>(get_param("encoder_telemetry_hz", 75.0));
    imu_telemetry_hz_ = static_cast<uint32_t>(get_param("imu_telemetry_hz", 100.0));
    battery_telemetry_hz_ = static_cast<uint32_t>(get_param("battery_telemetry_hz", 1.0));
  encoder_startup_timeout_ =
      std::chrono::milliseconds(static_cast<int>(get_param("encoder_startup_timeout_ms", 750.0)));
  encoder_stale_timeout_ =
      std::chrono::milliseconds(static_cast<int>(get_param("encoder_stale_timeout_ms", 250.0)));
    sensor_startup_timeout_ =
      std::chrono::milliseconds(static_cast<int>(get_param("sensor_startup_timeout_ms", 1000.0)));
  (void)params;  // no zero-cross params by default

  // -----------------------------------------------------------------------
  // Motor LUTs: {pwm, rad_s} pairs for the positive direction, sorted by
  // pwm ascending.  The first entry is {deadzone_pwm, 0.0}.
  //
  // Replace these placeholder entries with the output of motor_diagnostic.py
  // after running a calibration sweep on the actual hardware.
  // -----------------------------------------------------------------------
  motor_lut_[0] = {
      // left motor
      {0.1374, 0.0000},  // deadzone — minimum PWM to produce motion
      {0.3000, 1.9409}, {0.4000, 3.3857}, {0.5000, 4.8368}, {0.6000, 6.2393},
      {0.7000, 7.6250}, {0.8000, 8.6263}, {0.9000, 9.2348}, {1.0000, 11.5892},
  };
  motor_lut_[1] = {
      // right motor
      {0.0581, 0.0000},  // deadzone — minimum PWM to produce motion
      {0.3000, 2.2495}, {0.4000, 3.4411}, {0.5000, 4.6574}, {0.6000, 5.8840},
      {0.7000, 6.8992}, {0.8000, 7.6071}, {0.9000, 8.1498}, {1.0000, 9.8221},
  };

  // Sanitize: drop any LUT entry whose rad_s does not strictly exceed the
  // previous entry's rad_s.  Non-monotonic data (e.g. from a bad measurement
  // at a high PWM step) would otherwise corrupt the interpolation and clamp.
  for (auto& lut : motor_lut_) {
    double prev_rad_s = -1.0;
    std::vector<std::pair<double, double>> clean;
    for (const auto& entry : lut) {
      if (entry.second > prev_rad_s) {
        clean.push_back(entry);
        prev_rad_s = entry.second;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("Rover2SystemHardware"),
                    "LUT entry {pwm=%.4f, rad_s=%.4f} dropped: rad_s not greater than previous "
                    "entry (%.4f). Check motor_diagnostic output for bad data points.",
                    entry.first, entry.second, prev_rad_s);
      }
    }
    lut = std::move(clean);
  }

  controller_namespace_ = info_.hardware_parameters.count("controller_namespace")
                              ? info_.hardware_parameters["controller_namespace"]
                              : "rover2_base_controller";

  bool use_mock_hw = info_.hardware_parameters["use_mock_hardware"] == "true";
  if (use_mock_hw) {
    RCLCPP_WARN(rclcpp::get_logger("Rover2SystemHardware"),
                "Using mock hardware. No actual connection to RP2040 will be made.");
  } else {
    // Initialize RP2040 bridge for motor control and encoder reading
    try {
      auto serial_port = info_.hardware_parameters.count("serial_port")
                             ? info_.hardware_parameters["serial_port"]
                             : "/dev/ttyS0";
      auto baud_rate = info_.hardware_parameters.count("baud_rate")
                           ? stoi(info_.hardware_parameters["baud_rate"])
                           : 115200;
      bridge_ = std::make_shared<Rp2040Bridge>(serial_port, baud_rate);

        bool use_sim_time = info_.hardware_parameters.count("use_sim_time") &&
                  info_.hardware_parameters.at("use_sim_time") == "true";

      sensor_node_ =
          std::make_shared<Rp2040SensorNode>(bridge_, controller_namespace_, use_sim_time);

      RCLCPP_INFO(rclcpp::get_logger("Rover2SystemHardware"),
                  "RP2040 bridge connected successfully.");
    } catch (std::exception& ex) {
      RCLCPP_FATAL(rclcpp::get_logger("Rover2SystemHardware"),
                   "Error while initializing RP2040 bridge: '%s'", ex.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Rover2SystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Rover2SystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Rover2SystemHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (auto i = 0u; i < hw_positions_.size(); i++) {
    hw_positions_[i] = 0;
    hw_commands_[i] = 0;
    hw_velocities_[i] = 0;
  }

  if (!bridge_->stopMotors()) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2SystemHardware"),
                 "Error stopping motors while activating!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset encoder counts on the RP2040 so odometry starts from zero.
  if (!bridge_->resetEncoders()) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2SystemHardware"),
                 "Failed to reset encoders on RP2040 during activation!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!configure_encoder_telemetry()) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2SystemHardware"),
                 "Failed to receive fresh encoder telemetry during activation!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!configure_sensor_telemetry()) {
    RCLCPP_WARN(rclcpp::get_logger("Rover2SystemHardware"),
                "IMU telemetry was not observed during activation. Check RP2040 sensor wiring "
                "and serial telemetry setup if `/rover2_base_controller/imu` stays empty.");
  }

  start_sensor_executor();

  RCLCPP_INFO(rclcpp::get_logger("Rover2SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rover2SystemHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  stop_sensor_executor();

  if (!bridge_->stopMotors()) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2SystemHardware"),
                 "Error stopping motors while deactivating!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Rover2SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Rover2SystemHardware::read(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& period) {
  EncoderData enc_data;
  if (!bridge_->readEncoders(enc_data)) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2SystemHardware"),
                 "Failed to read encoders from RP2040!");
    return hardware_interface::return_type::ERROR;
  }

  int64_t ticks[2] = {enc_data.left_ticks, enc_data.right_ticks};

  double dt = period.seconds();
  if (dt <= 0.0) dt = 1e-6;

  for (auto i = 0u; i < info_.joints.size() && i < 2; i++) {
    int64_t delta = ticks[i] - last_tick_counts_[i];
    last_tick_counts_[i] = ticks[i];

    // Single-pin encoder can't detect direction; infer purely from commanded velocity.
    if (hw_commands_[i] < 0.0) {
      last_direction_[i] = -1;
    } else if (hw_commands_[i] > 0.0) {
      last_direction_[i] = 1;
    }

    if (last_direction_[i] < 0) {
      delta = -delta;
    }

    // Convert tick delta to radians and accumulate position.
    double delta_rad = (static_cast<double>(delta) / ticks_per_rev_[i]) * 2.0 * M_PI;
    hw_positions_[i] += delta_rad;
    hw_velocities_[i] = delta_rad / dt;
  }

  if (sensor_node_) {
    sensor_node_->updateMeasuredVelocities(
        hw_velocities_.size() > 0 ? hw_velocities_[0] : 0.0,
        hw_velocities_.size() > 1 ? hw_velocities_[1] : 0.0);
  }

  return hardware_interface::return_type::OK;
}

bool Rover2SystemHardware::configure_encoder_telemetry() {
  if (!bridge_) {
    return false;
  }

  if (!bridge_->setTelemetryRate('E', encoder_telemetry_hz_)) {
    RCLCPP_WARN(rclcpp::get_logger("Rover2SystemHardware"),
                "Failed to send encoder telemetry rate command; relying on firmware default rate.");
  }

  EncoderData enc_data;
  uint32_t seq = 0;
  std::chrono::steady_clock::time_point ts;
  if (!bridge_->waitForFreshEncoderTelemetry(enc_data, seq, ts, encoder_startup_timeout_,
                                             encoder_stale_timeout_)) {
    return false;
  }

  last_tick_counts_[0] = enc_data.left_ticks;
  last_tick_counts_[1] = enc_data.right_ticks;
  return true;
}

bool Rover2SystemHardware::configure_sensor_telemetry() {
  if (!bridge_) {
    return false;
  }

  if (imu_telemetry_hz_ > 0 && !bridge_->setTelemetryRate('I', imu_telemetry_hz_)) {
    RCLCPP_WARN(rclcpp::get_logger("Rover2SystemHardware"),
                "Failed to send IMU telemetry rate command; relying on firmware default rate.");
  }

  if (battery_telemetry_hz_ > 0 && !bridge_->setTelemetryRate('B', battery_telemetry_hz_)) {
    RCLCPP_WARN(rclcpp::get_logger("Rover2SystemHardware"),
                "Failed to send battery telemetry rate command; relying on firmware default rate.");
  }

  if (imu_telemetry_hz_ == 0) {
    return true;
  }

  auto deadline = std::chrono::steady_clock::now() + sensor_startup_timeout_;
  while (std::chrono::steady_clock::now() < deadline) {
    ImuData imu;
    std::chrono::steady_clock::time_point imu_ts;
    if (bridge_->getLatestImuSnapshot(imu, imu_ts)) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }

  return false;
}

void Rover2SystemHardware::start_sensor_executor() {
  if (!sensor_node_ || sensor_executor_running_) {
    return;
  }

  sensor_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  sensor_executor_->add_node(sensor_node_);
  sensor_executor_running_ = true;
  sensor_thread_ = std::thread([this]() {
    while (sensor_executor_running_ && rclcpp::ok()) {
      sensor_executor_->spin_some();
      std::this_thread::sleep_for(2ms);
    }
  });
}

void Rover2SystemHardware::stop_sensor_executor() {
  if (!sensor_executor_running_) {
    return;
  }

  sensor_executor_running_ = false;
  if (sensor_executor_) {
    sensor_executor_->cancel();
  }
  if (sensor_thread_.joinable()) {
    sensor_thread_.join();
  }
  if (sensor_executor_ && sensor_node_) {
    sensor_executor_->remove_node(sensor_node_);
  }
  sensor_executor_.reset();
}

hardware_interface::return_type Rover2SystemHardware::write(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/) {
  double left_rad_s = hw_commands_.size() > 0 ? hw_commands_[0] : 0.0;
  double right_rad_s = hw_commands_.size() > 1 ? hw_commands_[1] : 0.0;
  (void)left_rad_s;
  (void)right_rad_s;  // no temporary deadband/ramp/accel logic

  // Convert rad/s to PWM via LUT interpolation.
  // lut entries: {pwm, rad_s}, sorted by pwm ascending; lut[0] = {deadzone_pwm, 0.0}.
  // For commanded speeds below the minimum moving speed, the deadzone PWM is applied
  // so the motor receives at least the minimum effort needed to turn.
  // For commanded speeds above the maximum measured speed, the PWM is clamped to 1.0.
  auto pwm_from_lut = [](const std::vector<std::pair<double, double>>& lut,
                         double abs_rad_s) -> double {
    if (abs_rad_s < 1e-6 || lut.empty()) return 0.0;
    // Below (or at) first measured moving speed -> return deadzone PWM.
    if (abs_rad_s <= lut[0].second) return lut[0].first;
    // Above maximum measured speed -> clamp to full PWM.
    if (abs_rad_s >= lut.back().second) return 1.0;
    // Linear interpolation between bracketing entries.
    for (size_t i = 0; i + 1 < lut.size(); ++i) {
      if (lut[i].second <= abs_rad_s && abs_rad_s < lut[i + 1].second) {
        double t = (abs_rad_s - lut[i].second) / (lut[i + 1].second - lut[i].second);
        return lut[i].first + t * (lut[i + 1].first - lut[i].first);
      }
    }
    return lut.back().first;
  };

  auto apply_lut = [&](double rad_s, size_t idx) -> double {
    if (std::abs(rad_s) < 1e-6) return 0.0;
    double sign = rad_s > 0.0 ? 1.0 : -1.0;
    return sign * pwm_from_lut(motor_lut_[idx], std::abs(rad_s));
  };

  double left_pwm = apply_lut(left_rad_s, 0);
  double right_pwm = apply_lut(right_rad_s, 1);

  if (sensor_node_) {
    sensor_node_->updateCommandedVelocities(left_rad_s, right_rad_s);
  }

  // Guard against micro PWM commands that cause motor/driver jitter.
  // If the interpolated PWM is extremely small, send exact zero instead
  // so the RP2040 and motor drivers don't toggle micro-commands.
  const double pwm_clamp_eps = 1e-3;
  if (std::abs(left_pwm) < pwm_clamp_eps) left_pwm = 0.0;
  if (std::abs(right_pwm) < pwm_clamp_eps) right_pwm = 0.0;

  (void)left_pwm;
  (void)right_pwm;

  if (!bridge_->setMotorVelocities(left_pwm, right_pwm)) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2SystemHardware"),
                 "Error setting motor velocities via RP2040!");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(viam_rover2::Rover2SystemHardware, hardware_interface::SystemInterface)
