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
// (mikelikesrobots@outlook.com), on 2024-03-07.

#ifndef viam_rover2__rover2_system_HPP_
#define viam_rover2__rover2_system_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "viam_rover2/rp2040_bridge.hpp"
#include "viam_rover2/rp2040_sensor_node.hpp"
#include "viam_rover2/visibility_control.h"

namespace viam_rover2 {
class Rover2SystemHardware : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Rover2SystemHardware)

    ~Rover2SystemHardware() override;

  VIAM_ROVER2_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;

  VIAM_ROVER2_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  VIAM_ROVER2_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  VIAM_ROVER2_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  VIAM_ROVER2_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  VIAM_ROVER2_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  VIAM_ROVER2_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  void start_sensor_executor();
  void stop_sensor_executor();
  bool configure_encoder_telemetry();
  bool configure_sensor_telemetry();

  std::shared_ptr<Rp2040Bridge> bridge_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::vector<int64_t> last_tick_counts_;
  std::vector<int> last_direction_;
  std::vector<double> ticks_per_rev_;

  // Per-motor LUT: each entry is {pwm, rad_s} for the positive direction,
  // sorted by pwm ascending.  motor_lut_[0] = left, motor_lut_[1] = right.
  // The first entry must be {deadzone_pwm, 0.0}.
  // Replace the placeholder entries in on_init() with output from motor_diagnostic.py.
  std::array<std::vector<std::pair<double, double>>, 2> motor_lut_;
  uint32_t encoder_telemetry_hz_{75};
  uint32_t imu_telemetry_hz_{100};
  uint32_t battery_telemetry_hz_{1};
  std::chrono::milliseconds encoder_startup_timeout_{750};
  std::chrono::milliseconds encoder_stale_timeout_{250};
  std::chrono::milliseconds sensor_startup_timeout_{1000};
  std::string controller_namespace_;
  std::shared_ptr<Rp2040SensorNode> sensor_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> sensor_executor_;
  std::thread sensor_thread_;
  bool sensor_executor_running_{false};
};

}  // namespace viam_rover2

#endif  // viam_rover2__rover2_system_HPP_
