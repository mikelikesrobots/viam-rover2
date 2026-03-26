#include "viam_rover2/rp2040_bridge.hpp"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace viam_rover2;

namespace {

speed_t to_speed(int baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

bool is_async_info_line(const std::string& line) {
  return line.rfind("I2C_PINS ", 0) == 0 || line.rfind("SENSOR ", 0) == 0;
}

}  // namespace

Rp2040Bridge::Rp2040Bridge(const std::string& port, int baud_rate) : uart_fd_{-1}, connected_{false} {
  uart_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (uart_fd_ < 0) {
    throw std::runtime_error("Failed to open serial port: " + port);
  }

  // Configure termios: raw mode, 8N1, no flow control
  struct termios tty{};
  if (tcgetattr(uart_fd_, &tty) != 0) {
    close(uart_fd_);
    throw std::runtime_error("Failed to get terminal attributes for: " + port);
  }

  cfmakeraw(&tty);
  speed_t speed = to_speed(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;  // no hw flow control
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
    close(uart_fd_);
    throw std::runtime_error("Failed to set terminal attributes for: " + port);
  }

  // Flush any stale data
  tcflush(uart_fd_, TCIOFLUSH);
  flushInput();
  connected_ = true;

  // Start reader thread (single owner of serial reads)
  stop_reader_.store(false);
  reader_thread_ = std::thread(&Rp2040Bridge::readerLoop, this);

  // Verify RP2040 is alive (reader thread will capture responses). The board
  // can take a moment to start answering after the port is opened, so retry for
  // a short bounded window instead of failing after a single fast round-trip.
  bool ping_ok = false;
  for (int attempt = 0; attempt < 8; ++attempt) {
    if (ping()) {
      ping_ok = true;
      break;
    }
    usleep(200000);
    flushInput();
  }
  if (!ping_ok) {
    stop_reader_.store(true);
    rx_cv_.notify_all();
    if (reader_thread_.joinable()) reader_thread_.join();
    close(uart_fd_);
    uart_fd_ = -1;
    connected_ = false;
    throw std::runtime_error("RP2040 not responding on: " + port +
                             ". Ensure firmware is flashed and running.");
  }
}

Rp2040Bridge::~Rp2040Bridge() {
  if (connected_) {
    stopMotors();
  }
  if (uart_fd_ >= 0) {
    // Stop reader thread cleanly
    stop_reader_.store(true);
    rx_cv_.notify_all();
    if (reader_thread_.joinable()) reader_thread_.join();

    close(uart_fd_);
  }
}

bool Rp2040Bridge::sendCommand(const std::string& cmd) {
  if (uart_fd_ < 0) return false;
  std::string msg = cmd + "\r\n";
  ssize_t written = write(uart_fd_, msg.c_str(), msg.size());
  return written == static_cast<ssize_t>(msg.size());
}

bool Rp2040Bridge::readResponse(std::string& response, int timeout_ms) {
  if (uart_fd_ < 0) return false;

  response.clear();
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  std::unique_lock<std::mutex> lk(rx_mutex_);
  while (true) {
    // Find first command-response line in queue. Skip telemetry and async
    // startup info banners emitted by the RP2040 firmware.
    for (auto it = rx_queue_.begin(); it != rx_queue_.end(); ++it) {
      if (it->empty()) continue;
      if (((*it).size() >= 2 && (*it)[0] == 'T' && (*it)[1] == ' ') || (*it)[0] == '\x02') {
        continue;  // telemetry frame
      }
      if (is_async_info_line(*it)) {
        it = rx_queue_.erase(it);
        if (it == rx_queue_.end()) {
          break;
        }
        --it;
        continue;
      }
      response = *it;
      rx_queue_.erase(it);
      return true;
    }

    if (timeout_ms <= 0) return false;
    if (rx_cv_.wait_until(lk, deadline) == std::cv_status::timeout) return false;
    // loop and re-check queue
  }
}

void Rp2040Bridge::flushInput() {
  if (uart_fd_ < 0) return;
  char buf[64];
  // Non-blocking read to drain any buffered data
  while (read(uart_fd_, buf, sizeof(buf)) > 0) {
  }
}

bool Rp2040Bridge::setMotorVelocities(double left, double right) {
  left = std::max(-1.0, std::min(1.0, left));
  right = std::max(-1.0, std::min(1.0, right));

  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "M %.3f %.3f", left, right);

  if (!sendCommand(cmd)) return false;

  std::string response;
  if (!readResponse(response, 300)) return false;
  return response == "OK";
}

bool Rp2040Bridge::setTelemetryRate(char stream, uint32_t hz) {
  if (stream != 'E' && stream != 'I' && stream != 'B') return false;

  char cmd[32];
  std::snprintf(cmd, sizeof(cmd), "TS %c %u", stream, hz);
  if (!sendCommand(cmd)) return false;

  std::string response;
  if (!readResponse(response, 300)) return false;
  return response == "OK";
}

bool Rp2040Bridge::stopMotors() {
  if (!sendCommand("S")) return false;
  std::string response;
  if (!readResponse(response, 300)) return false;
  return response == "OK";
}

bool Rp2040Bridge::resetEncoders() {
  if (!sendCommand("R")) return false;
  std::string response;
  if (!readResponse(response, 300)) return false;
  return response == "OK";
}

bool Rp2040Bridge::ping() {
  if (!sendCommand("P")) return false;
  std::string response;
  if (!readResponse(response, 500)) return false;
  return response == "P";
}

bool Rp2040Bridge::isConnected() const { return connected_; }

// Compute simple 8-bit XOR CRC over payload bytes.
static uint8_t compute_crc_xor(const std::string& payload) {
  uint8_t crc = 0;
  for (unsigned char c : payload) crc ^= c;
  return crc;
}

// Read a 32-bit little-endian integer from a byte pointer.
static uint32_t read_le32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

// Read a little-endian IEEE-754 float from a byte pointer.
static float read_le_float(const uint8_t* p) {
  uint32_t bits = read_le32(p);
  float f;
  memcpy(&f, &bits, sizeof(float));
  return f;
}

void Rp2040Bridge::readerLoop() {
  if (uart_fd_ < 0) return;

  char tmp[256];
  std::vector<uint8_t> rawbuf;  // accumulates partial frames across reads

  while (!stop_reader_.load()) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(uart_fd_, &rfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000;  // 200 ms

    int ret = select(uart_fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
      continue;
    }

    ssize_t n = read(uart_fd_, tmp, sizeof(tmp));
    if (n <= 0) continue;

    // Append received bytes and parse binary frames (falling back to ASCII lines).
    for (ssize_t i = 0; i < n; ++i) rawbuf.push_back(static_cast<uint8_t>(tmp[i]));

    while (!rawbuf.empty()) {
      // If frame starts with STX (binary frame)
      if (rawbuf[0] == 0x02) {
        if (rawbuf.size() < 2) break;  // need length
        uint8_t payload_len = rawbuf[1];
        size_t total = 2 + (size_t)payload_len + 1;  // STX + LEN + PAYLOAD + CRC
        if (rawbuf.size() < total) break;            // wait for full frame
        // extract payload
        const uint8_t* payload = rawbuf.data() + 2;
        uint8_t crc = rawbuf[2 + payload_len];
        uint8_t seen = 0;
        for (size_t j = 0; j < payload_len; ++j) seen ^= payload[j];
        if (seen != crc) {
          // discard this STX and continue
          rawbuf.erase(rawbuf.begin());
          continue;
        }

        // parse payload: first byte is type
        char type = static_cast<char>(payload[0]);
        if (type == 'E') {
          // E: type(1) + seq(4) + left_ticks(4) + right_ticks(4)
          if (payload_len >= 1 + 4 + 4 + 4) {
            const uint8_t* p = payload + 1;
            uint32_t seq   = read_le32(p);
            int32_t  left  = static_cast<int32_t>(read_le32(p + 4));
            int32_t  right = static_cast<int32_t>(read_le32(p + 8));
            std::lock_guard<std::mutex> lk(telemetry_mutex_);
            buffered_encoders_.left_ticks = static_cast<int64_t>(left);
            buffered_encoders_.right_ticks = static_cast<int64_t>(right);
            encoder_seq_ = seq;
            last_encoder_time_ = std::chrono::steady_clock::now();
            telemetry_cv_.notify_all();
          }
        } else if (type == 'I') {
          // I: type(1) + seq(4) + ax(4) + ay(4) + az(4) + gx(4) + gy(4) + gz(4)
          if (payload_len >= 1 + 4 + 6 * 4) {
            const uint8_t* p = payload + 1;
            uint32_t seq = read_le32(p);
            float ax = read_le_float(p + 4);
            float ay = read_le_float(p + 8);
            float az = read_le_float(p + 12);
            float gx = read_le_float(p + 16);
            float gy = read_le_float(p + 20);
            float gz = read_le_float(p + 24);
            std::lock_guard<std::mutex> lk(telemetry_mutex_);
            buffered_imu_.seq = seq;
            buffered_imu_.ax = ax;
            buffered_imu_.ay = ay;
            buffered_imu_.az = az;
            buffered_imu_.gx = gx;
            buffered_imu_.gy = gy;
            // RP2040 IMU telemetry yaw has the opposite sign from the rover's
            // odom/ROS convention. Correct it at ingest so downstream consumers
            // all see a consistent angular velocity about +Z.
            buffered_imu_.gz = -gz;
            last_imu_time_ = std::chrono::steady_clock::now();
          }
        } else if (type == 'B') {
          // B: type(1) + seq(4) + voltage_mV(4) + current_mA(4) + soc(4 float)
          if (payload_len >= 1 + 4 + 4 + 4 + 4) {
            const uint8_t* p = payload + 1;
            uint32_t seq  = read_le32(p);
            int32_t  volt = static_cast<int32_t>(read_le32(p + 4));
            int32_t  curr = static_cast<int32_t>(read_le32(p + 8));
            float    soc  = read_le_float(p + 12);
            std::lock_guard<std::mutex> lk(telemetry_mutex_);
            buffered_battery_.seq = seq;
            buffered_battery_.voltage_mV = volt;
            buffered_battery_.current_mA = curr;
            buffered_battery_.soc = soc;
            last_battery_time_ = std::chrono::steady_clock::now();
          }
        }

        // consume frame
        rawbuf.erase(rawbuf.begin(), rawbuf.begin() + total);
        continue;
      }

      // Not an STX frame: try to parse an ASCII line terminated by LF
      auto itlf = std::find(rawbuf.begin(), rawbuf.end(), (uint8_t)'\n');
      if (itlf == rawbuf.end()) break;
      // build string line
      std::string line(rawbuf.begin(), itlf);
      // remove possible CR
      if (!line.empty() && line.back() == '\r') line.pop_back();

      // Try to parse telemetry-like ASCII (legacy) starting with 'T '
      if (line.size() >= 2 && line[0] == 'T' && line[1] == ' ') {
        auto last_sp = line.find_last_of(' ');
        if (last_sp != std::string::npos) {
          std::string payload = line.substr(0, last_sp);
          std::string crc_token = line.substr(last_sp + 1);
          uint8_t expected_crc = 0;
          try {
            expected_crc = static_cast<uint8_t>(std::stoul(crc_token));
          } catch (...) {
            // malformed
            rawbuf.erase(rawbuf.begin(), itlf + 1);
            continue;
          }
          uint8_t seen_crc = compute_crc_xor(payload);
          if (seen_crc == expected_crc) {
            std::istringstream iss(payload);
            std::string lead;
            iss >> lead;
            std::string type;
            iss >> type;
            if (type == "E") {
              uint32_t seq;
              long long left, right;
              if (iss >> seq >> left >> right) {
                std::lock_guard<std::mutex> lk(telemetry_mutex_);
                buffered_encoders_.left_ticks = static_cast<int64_t>(left);
                buffered_encoders_.right_ticks = static_cast<int64_t>(right);
                encoder_seq_ = seq;
                last_encoder_time_ = std::chrono::steady_clock::now();
              }
            }
          }
        }
      } else {
        // Non-telemetry ASCII -> push to rx_queue_
        {
          std::lock_guard<std::mutex> lk(rx_mutex_);
          rx_queue_.push_back(line);
        }
        rx_cv_.notify_all();
      }

      // consume up to LF
      rawbuf.erase(rawbuf.begin(), itlf + 1);
    }
  }
}

bool Rp2040Bridge::getLatestEncoderSnapshot(EncoderData& out, uint32_t& seq,
                                            std::chrono::steady_clock::time_point& ts) {
  std::lock_guard<std::mutex> lk(telemetry_mutex_);
  if (last_encoder_time_.time_since_epoch().count() == 0) return false;
  out = buffered_encoders_;
  seq = encoder_seq_;
  ts = last_encoder_time_;
  return true;
}

bool Rp2040Bridge::waitForFreshEncoderTelemetry(EncoderData& out, uint32_t& seq,
                                                std::chrono::steady_clock::time_point& ts,
                                                std::chrono::milliseconds timeout,
                                                std::chrono::milliseconds max_age) {
  std::unique_lock<std::mutex> lk(telemetry_mutex_);
  auto deadline = std::chrono::steady_clock::now() + timeout;

  while (true) {
    if (last_encoder_time_.time_since_epoch().count() != 0) {
      auto age = std::chrono::steady_clock::now() - last_encoder_time_;
      if (age <= max_age) {
        out = buffered_encoders_;
        seq = encoder_seq_;
        ts = last_encoder_time_;
        return true;
      }
    }

    if (telemetry_cv_.wait_until(lk, deadline) == std::cv_status::timeout) {
      return false;
    }
  }
}

bool Rp2040Bridge::getLatestImuSnapshot(ImuData& out, std::chrono::steady_clock::time_point& ts) {
  std::lock_guard<std::mutex> lk(telemetry_mutex_);
  if (last_imu_time_.time_since_epoch().count() == 0) return false;
  out = buffered_imu_;
  ts = last_imu_time_;
  return true;
}

bool Rp2040Bridge::getLatestBatterySnapshot(BatteryData& out,
                                            std::chrono::steady_clock::time_point& ts) {
  std::lock_guard<std::mutex> lk(telemetry_mutex_);
  if (last_battery_time_.time_since_epoch().count() == 0) return false;
  out = buffered_battery_;
  ts = last_battery_time_;
  return true;
}

bool Rp2040Bridge::readEncoders(EncoderData& data) {
  std::lock_guard<std::mutex> lk(telemetry_mutex_);
  if (last_encoder_time_.time_since_epoch().count() == 0) return false;

  auto age = std::chrono::steady_clock::now() - last_encoder_time_;
  if (age > std::chrono::milliseconds(200)) return false;

  data = buffered_encoders_;
  return true;
}
