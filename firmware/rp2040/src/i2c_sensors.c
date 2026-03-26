#include "i2c_sensors.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

// Fixed I2C bus: i2c0, GPIO28 (SDA), GPIO29 (SCL)
#define I2C_SDA_PIN 28
#define I2C_SCL_PIN 29

// Fixed sensor addresses
#define MPU6050_ADDR 0x68
#define INA219_ADDR 0x40

// Shunt resistor used for INA219 current calc (ohms)
#ifndef INA219_SHUNT_OHM
#define INA219_SHUNT_OHM 0.1f
#endif

// Battery voltage range for SoC calculation (4S LiPo: 14.8 V empty, 16.8 V full)
#ifndef BATTERY_MIN_MV
#define BATTERY_MIN_MV 14800
#endif
#ifndef BATTERY_MAX_MV
#define BATTERY_MAX_MV 16800
#endif

// MPU6050 registers
#define MPUREG_PWR_MGMT_1 0x6B
#define MPUREG_ACCEL_XOUT_H 0x3B

// INA219 registers
#define INA_REG_SHUNTV 0x01
#define INA_REG_BUSV 0x02

#define I2C_IO_TIMEOUT_US 3000

static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;
static float accel_bias_x = 0.0f;
static float accel_bias_y = 0.0f;
static float accel_bias_z = 0.0f;
static bool sensors_initialized = false;

static bool i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
  uint8_t buf[1 + len];
  buf[0] = reg;
  memcpy(&buf[1], data, len);
  int ret = i2c_write_timeout_us(i2c0, addr, buf, 1 + len, false, I2C_IO_TIMEOUT_US);
  return ret >= 0;
}

static bool i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t* out, size_t len) {
  int ret = i2c_write_timeout_us(i2c0, addr, &reg, 1, true, I2C_IO_TIMEOUT_US);
  if (ret < 0) return false;
  ret = i2c_read_timeout_us(i2c0, addr, out, len, false, I2C_IO_TIMEOUT_US);
  return ret >= 0;
}

static bool i2c_get_reading(imu_reading_t* reading) {
    uint8_t buf[14];
  if (!i2c_read_reg(MPU6050_ADDR, MPUREG_ACCEL_XOUT_H, buf, sizeof(buf))) return false;

  int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
  int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
  int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
  int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

  reading->ax = (float)raw_ax;
  reading->ay = (float)raw_ay;
  reading->az = (float)raw_az;
  reading->gx = (float)raw_gx;
  reading->gy = (float)raw_gy;
  reading->gz = (float)raw_gz;

  return true;
}

void sensors_init(void) {
  if (sensors_initialized) return;

  i2c_init(i2c0, 100000);
  gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA_PIN);
  gpio_pull_up(I2C_SCL_PIN);
  sleep_ms(2);

  // Wake MPU6050 and wait for it to stabilise.  The datasheet requires ~30 ms
  // after clearing SLEEP before the internal oscillator is ready and sensor
  // output is valid.
  uint8_t zero = 0x00;
  i2c_write_reg(MPU6050_ADDR, MPUREG_PWR_MGMT_1, &zero, 1);
  sleep_ms(30);

  // Compute gyro zero-bias from averaged samples at rest
  const size_t samples = 200;
  float ax_sum = 0.0f, ay_sum = 0.0f, az_sum = 0.0f;
  float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
  imu_reading_t reading;
  size_t valid = 0;
  for (size_t i = 0; i < samples; ++i) {
    if (!i2c_get_reading(&reading)) {
      sleep_ms(5);
      continue;
    }
    ax_sum += reading.ax;
    ay_sum += reading.ay;
    az_sum += reading.az;
    gx_sum += reading.gx;
    gy_sum += reading.gy;
    gz_sum += reading.gz;
    valid++;
    sleep_ms(5);
  }

  if (valid > 0) {
    // Capture resting accelerometer baseline (includes gravity projection due to
    // mounting angle) so reported linear acceleration is near 0 when stationary.
    accel_bias_x = ax_sum / (float)valid;
    accel_bias_y = ay_sum / (float)valid;
    accel_bias_z = az_sum / (float)valid;

    gyro_bias_x = gx_sum / (float)valid;
    gyro_bias_y = gy_sum / (float)valid;
    gyro_bias_z = gz_sum / (float)valid;
  }

  sensors_initialized = true;
}

bool sensors_read_imu(imu_reading_t* reading) {
  if (!sensors_initialized) sensors_init();

  if (!i2c_get_reading(reading)) return false;

  // MPU6050 default scale: accel LSB = 16384 per g, gyro LSB = 131 per deg/s
  const float accel_lsb = 16384.0f;
  const float gyro_lsb = 131.0f;

  const float G = 9.80665f;
  // convert to m/s^2 and rad/s
  // subtract resting baseline (in raw accel LSB) before converting to m/s^2
  reading->ax = ((reading->ax) - accel_bias_x) / accel_lsb * G;
  reading->ay = ((reading->ay) - accel_bias_y) / accel_lsb * G;
  reading->az = ((reading->az) - accel_bias_z) / accel_lsb * G;

  // subtract bias (bias is in raw gyro LSB units)
  float gx_f = (reading->gx - gyro_bias_x) / gyro_lsb;  // deg/s
  float gy_f = (reading->gy - gyro_bias_y) / gyro_lsb;
  float gz_f = (reading->gz - gyro_bias_z) / gyro_lsb;
  const float d2r = 3.14159265358979323846f / 180.0f;
  reading->gx = gx_f * d2r;
  reading->gy = gy_f * d2r;
  reading->gz = gz_f * d2r;

  return true;
}

bool sensors_read_battery(battery_status_t* status) {
  if (!sensors_initialized) sensors_init();

  uint8_t buf[2];
  // read shunt voltage (signed 16-bit, LSB = 10uV)
  if (!i2c_read_reg(INA219_ADDR, INA_REG_SHUNTV, buf, 2)) return false;
  int16_t shunt_raw = (int16_t)((buf[0] << 8) | buf[1]);
  int32_t shunt_uV = (int32_t)shunt_raw * 10;  // 10 uV LSB

  // read bus voltage (16-bit; bits [15:3] are voltage, LSB = 4mV)
  if (!i2c_read_reg(INA219_ADDR, INA_REG_BUSV, buf, 2)) return false;
  uint16_t bus = (uint16_t)((buf[0] << 8) | buf[1]);
  uint16_t bus_v_raw = (bus >> 3);
  int32_t bus_mV = (int32_t)bus_v_raw * 4;  // 4 mV LSB

  // current estimated from shunt voltage / R_shunt
  float current_A = (float)shunt_uV / 1e6f / INA219_SHUNT_OHM;

  status->voltage_mV = bus_mV;
  status->current_mA = (int)(current_A * 1000.0f);

  // SoC estimate for 4S LiPo: 14.8 V (empty) -> 16.8 V (full)
  const float v_min = (float)BATTERY_MIN_MV;
  const float v_max = (float)BATTERY_MAX_MV;
  float v = (float)bus_mV;
  float frac = (v - v_min) / (v_max - v_min);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  status->soc = frac * 100.0f;

  return true;
}

int sensors_get_sda_pin(void) { return I2C_SDA_PIN; }
int sensors_get_scl_pin(void) { return I2C_SCL_PIN; }
int sensors_get_bus_num(void) { return 0; }
int sensors_get_mpu_address(void) { return MPU6050_ADDR; }
int sensors_get_ina_address(void) { return INA219_ADDR; }
