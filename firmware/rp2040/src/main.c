/*
 * Rover2 RP2040 Firmware
 *
 * Handles motor PWM, encoder edge counting, and UART communication
 * with the host Intel N100 on the Radxa X4.
 *
 * Pin assignments:
 *   Left motor:  in1=GPIO15, in2=GPIO13, pwm=GPIO14
 *   Right motor: in1=GPIO12, in2=GPIO07, pwm=GPIO03
 *   Left encoder:  GPIO02
 *   Right encoder: GPIO25
 *   UART0: TX=GPIO0, RX=GPIO1 (to host /dev/ttyS0)
 */

#include "encoder.h"
#include "hardware/uart.h"
#include "i2c_sensors.h"
#include "motor.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "protocol.h"
#include "telemetry.h"

#define UART_ID uart0
#define UART_BAUD 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define LEFT_IN1_PIN 15
#define LEFT_IN2_PIN 13
#define LEFT_PWM_PIN 14

#define RIGHT_IN1_PIN 12
#define RIGHT_IN2_PIN 7
#define RIGHT_PWM_PIN 3

#define LEFT_ENC_PIN 2
#define RIGHT_ENC_PIN 25

#define WATCHDOG_TIMEOUT_MS 500

int main(void) {
  // Init UART
  uart_init(UART_ID, UART_BAUD);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_ID, true);

  // Init motors
  motor_t left_motor = {
      .in1_pin = LEFT_IN1_PIN,
      .in2_pin = LEFT_IN2_PIN,
      .pwm_pin = LEFT_PWM_PIN,
  };
  motor_init(&left_motor);

  motor_t right_motor = {
      .in1_pin = RIGHT_IN1_PIN,
      .in2_pin = RIGHT_IN2_PIN,
      .pwm_pin = RIGHT_PWM_PIN,
  };
  motor_init(&right_motor);

  // Init encoders
  encoder_init(LEFT_ENC_PIN, RIGHT_ENC_PIN);

  // Init protocol handler before sensors so the firmware can respond to host
  // pings immediately.  sensors_init() blocks for ~1-2 s during I2C calibration
  // and must not run before the UART protocol is ready.
  protocol_t proto = {
      .uart = UART_ID,
      .left_motor = &left_motor,
      .right_motor = &right_motor,
  };
  protocol_init(&proto);

  // Init telemetry before sensors for the same reason.
  telemetry_init(UART_ID);

  // Init I2C sensors (MPU6050, INA219) — blocking calibration loop (~1-2 s).
  sensors_init();

  absolute_time_t last_motor_cmd = get_absolute_time();

  while (true) {
    // Process incoming UART commands
    if ((protocol_poll(&proto) & PROT_MOTOR_CMD_RX) != 0) {
      last_motor_cmd = get_absolute_time();
    }

    // Poll telemetry broadcaster
    telemetry_poll();

    // Watchdog: stop motors if no command received recently
    if (absolute_time_diff_us(last_motor_cmd, get_absolute_time()) > WATCHDOG_TIMEOUT_MS * 1000) {
      motor_set(&left_motor, 0.0f);
      motor_set(&right_motor, 0.0f);
      last_motor_cmd = get_absolute_time();
    }

    sleep_ms(1);
  }

  return 0;
}
