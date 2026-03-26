#pragma once

#ifndef viam_rover2__RP2040_BRIDGE_HPP_
#define viam_rover2__RP2040_BRIDGE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace viam_rover2 {

struct EncoderData {
  int64_t left_ticks;
  int64_t right_ticks;
};

struct ImuData {
  uint32_t seq;
  double ax;
  double ay;
  double az;
  double gx;
  double gy;
  double gz;
};

struct BatteryData {
  uint32_t seq;
  int voltage_mV;
  int current_mA;
  float soc;
};

class Rp2040Bridge {
 public:
  explicit Rp2040Bridge(const std::string& port = "/dev/ttyS0",
                        int baud_rate = 115200);
  ~Rp2040Bridge();

  Rp2040Bridge(const Rp2040Bridge&) = delete;
  Rp2040Bridge& operator=(const Rp2040Bridge&) = delete;

  // Send motor velocity command. Values clamped to [-1.0, 1.0].
  bool setMotorVelocities(double left, double right);

  // Configure telemetry stream rate.
  bool setTelemetryRate(char stream, uint32_t hz);

  // Read encoder tick counts from buffered encoder telemetry.
  bool readEncoders(EncoderData& data);

  // Send stop command (zero both motors).
  bool stopMotors();

  // Check if the RP2040 is responsive.
  bool ping();

  // Reset encoder tick counters on the RP2040.
  bool resetEncoders();

  bool isConnected() const;

  // Real-time getters (non-blocking) for telemetry
  bool getLatestEncoderSnapshot(EncoderData& out, uint32_t& seq,
                                std::chrono::steady_clock::time_point& ts);
  bool waitForFreshEncoderTelemetry(EncoderData& out, uint32_t& seq,
                                    std::chrono::steady_clock::time_point& ts,
                                    std::chrono::milliseconds timeout,
                                    std::chrono::milliseconds max_age);
  bool getLatestImuSnapshot(ImuData& out,
                            std::chrono::steady_clock::time_point& ts);
  bool getLatestBatterySnapshot(BatteryData& out,
                                std::chrono::steady_clock::time_point& ts);

 private:
  bool sendCommand(const std::string& cmd);
  bool readResponse(std::string& response, int timeout_ms = 100);
  void flushInput();

  // Reader thread + RX queue for async line buffering
  void readerLoop();
  std::thread reader_thread_;
  std::atomic<bool> stop_reader_{false};
  std::mutex rx_mutex_;
  std::condition_variable rx_cv_;
  std::condition_variable telemetry_cv_;
  std::deque<std::string> rx_queue_;

  // Last buffered encoder telemetry (updated by reader thread)
  std::mutex telemetry_mutex_;
  EncoderData buffered_encoders_{0, 0};
  uint32_t encoder_seq_{0};
  ImuData buffered_imu_{};
  BatteryData buffered_battery_{};
  std::chrono::steady_clock::time_point last_encoder_time_;
  std::chrono::steady_clock::time_point last_imu_time_;
  std::chrono::steady_clock::time_point last_battery_time_;

  int uart_fd_;
  bool connected_;
};

}  // namespace viam_rover2

#endif  // viam_rover2__RP2040_BRIDGE_HPP_
