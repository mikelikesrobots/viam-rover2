#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float ax, ay, az;  // m/s^2
  float gx, gy, gz;  // rad/s
} imu_reading_t;

typedef struct {
  int voltage_mV;
  int current_mA;
  float soc;  // 0..100
} battery_status_t;

/**
 * Initialise I2C bus and sensors. This will also compute gyro bias and accel
 * baseline.
 */
void sensors_init(void);

/**
 * Reads IMU from MPU6050. Returns true on success, false on I2C error.
 * On success, fills struct with ax,ay,az in m/s^2 and gx,gy,gz in rad/s.
 */
bool sensors_read_imu(imu_reading_t* reading);

/**
 * Reads battery status. Returns true on success, false on I2C error.
 * On success, fills struct with voltage (mV), current (mA), soc (0..100).
 */
bool sensors_read_battery(battery_status_t* status);

/**
 * Get I2C pin and address information.
 */
int sensors_get_sda_pin(void);
int sensors_get_scl_pin(void);
int sensors_get_bus_num(void);
int sensors_get_mpu_address(void);
int sensors_get_ina_address(void);
