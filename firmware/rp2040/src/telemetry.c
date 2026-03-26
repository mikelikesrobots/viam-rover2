#include "telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "encoder.h"
#include "hardware/uart.h"
#include "i2c_sensors.h"
#include "pico/stdlib.h"
#include "pico/time.h"

static uart_inst_t* tele_uart = NULL;

static bool enc_enabled = true;
static bool imu_enabled = true;
static bool bat_enabled = true;

static uint32_t enc_seq = 0;
static uint32_t imu_seq = 0;
static uint32_t bat_seq = 0;

static uint32_t enc_interval_ms = 20;    // 50 Hz
static uint32_t imu_interval_ms = 10;    // 100 Hz
static uint32_t bat_interval_ms = 1000;  // 1 Hz

static uint64_t last_enc_ms = 0;
static uint64_t last_imu_ms = 0;
static uint64_t last_bat_ms = 0;

static uint8_t compute_crc_xor_buf(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) crc ^= data[i];
  return crc;
}

static void send_binary_frame(const uint8_t* payload, size_t payload_len) {
  if (!tele_uart) return;
  // Frame: STX(0x02) LEN(1) PAYLOAD(payload_len) CRC(1)
  uint8_t frame[1 + 1 + 255 + 1];
  if (payload_len > 255) return;
  frame[0] = 0x02;
  frame[1] = (uint8_t)payload_len;
  memcpy(&frame[2], payload, payload_len);
  uint8_t crc = compute_crc_xor_buf(payload, payload_len);
  frame[2 + payload_len] = crc;
  size_t total = 2 + payload_len + 1;
  uart_write_blocking(tele_uart, frame, total);
}

static void send_telemetry_enc(uint32_t seq, int32_t left, int32_t right) {
  uint8_t payload[1 + 4 + 4 + 4];
  size_t idx = 0;
  payload[idx++] = 'E';
  // seq little endian
  payload[idx++] = (uint8_t)(seq & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 24) & 0xFF);
  // left
  payload[idx++] = (uint8_t)(left & 0xFF);
  payload[idx++] = (uint8_t)((left >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((left >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((left >> 24) & 0xFF);
  // right
  payload[idx++] = (uint8_t)(right & 0xFF);
  payload[idx++] = (uint8_t)((right >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((right >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((right >> 24) & 0xFF);
  send_binary_frame(payload, idx);
}

static void send_telemetry_imu(uint32_t seq, imu_reading_t* imu_reading) {
  // pack as float32 for bandwidth
  uint8_t payload[1 + 4 + 6 * 4];
  size_t idx = 0;
  payload[idx++] = 'I';
  payload[idx++] = (uint8_t)(seq & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 24) & 0xFF);

  float fvals[6];
  fvals[0] = (float)imu_reading->ax;
  fvals[1] = (float)imu_reading->ay;
  fvals[2] = (float)imu_reading->az;
  fvals[3] = (float)imu_reading->gx;
  fvals[4] = (float)imu_reading->gy;
  fvals[5] = (float)imu_reading->gz;

  for (int i = 0; i < 6; ++i) {
    uint8_t* b = (uint8_t*)&fvals[i];
    // little-endian
    payload[idx++] = b[0];
    payload[idx++] = b[1];
    payload[idx++] = b[2];
    payload[idx++] = b[3];
  }
  send_binary_frame(payload, idx);
}

static void send_telemetry_bat(uint32_t seq, battery_status_t* status) {
  uint8_t payload[1 + 4 + 4 + 4 + 4];
  size_t idx = 0;
  payload[idx++] = 'B';
  payload[idx++] = (uint8_t)(seq & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((seq >> 24) & 0xFF);

  payload[idx++] = (uint8_t)(status->voltage_mV & 0xFF);
  payload[idx++] = (uint8_t)((status->voltage_mV >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((status->voltage_mV >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((status->voltage_mV >> 24) & 0xFF);

  payload[idx++] = (uint8_t)(status->current_mA & 0xFF);
  payload[idx++] = (uint8_t)((status->current_mA >> 8) & 0xFF);
  payload[idx++] = (uint8_t)((status->current_mA >> 16) & 0xFF);
  payload[idx++] = (uint8_t)((status->current_mA >> 24) & 0xFF);

  uint8_t* sb = (uint8_t*)&status->soc;
  payload[idx++] = sb[0];
  payload[idx++] = sb[1];
  payload[idx++] = sb[2];
  payload[idx++] = sb[3];
  send_binary_frame(payload, idx);
}

void telemetry_init(uart_inst_t* uart) {
  tele_uart = uart;
  last_enc_ms = to_ms_since_boot(get_absolute_time());
  last_imu_ms = last_enc_ms;
  last_bat_ms = last_enc_ms;
}

static void read_imu(imu_reading_t* reading) {
  if (!sensors_read_imu(reading)) {
    // fallback to simple gravity if sensor read fails
    reading->ax = 0.0;
    reading->ay = 0.0;
    reading->az = 9.81;
    reading->gx = 0.0;
    reading->gy = 0.0;
    reading->gz = 0.0;
  }
}

static void read_battery(battery_status_t* status) {
  if (!sensors_read_battery(status)) {
    status->voltage_mV = 12000;
    status->current_mA = 0;
    status->soc = 100.0f;
  }
}

void telemetry_poll(void) {
  uint64_t now = to_ms_since_boot(get_absolute_time());

  if (enc_enabled && (now - last_enc_ms) >= enc_interval_ms) {
    long left = encoder_get_left_ticks();
    long right = encoder_get_right_ticks();
    send_telemetry_enc(enc_seq++, (int32_t)left, (int32_t)right);
    last_enc_ms = now;
  }

  if (imu_enabled && (now - last_imu_ms) >= imu_interval_ms) {
    imu_reading_t reading;
    read_imu(&reading);
    send_telemetry_imu(imu_seq++, &reading);
    last_imu_ms = now;
  }

  if (bat_enabled && (now - last_bat_ms) >= bat_interval_ms) {
    battery_status_t status;
    read_battery(&status);
    send_telemetry_bat(bat_seq++, &status);
    last_bat_ms = now;
  }
}

void telemetry_set_rate(char stream, uint32_t hz) {
  if (hz == 0) {
    telemetry_enable(stream, false);
    return;
  }
  uint32_t interval = 1000 / hz;
  if (stream == 'E')
    enc_interval_ms = interval;
  else if (stream == 'I')
    imu_interval_ms = interval;
  else if (stream == 'B')
    bat_interval_ms = interval;
}

void telemetry_enable(char stream, bool enabled) {
  if (stream == 'E')
    enc_enabled = enabled;
  else if (stream == 'I')
    imu_enabled = enabled;
  else if (stream == 'B')
    bat_enabled = enabled;
}
