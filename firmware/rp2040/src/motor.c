#include "motor.h"

#include <math.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define PWM_WRAP 4999  // 125 MHz / (4999+1) = 25 kHz

void motor_init(motor_t* motor) {
  // Direction pins: digital output
  gpio_init(motor->in1_pin);
  gpio_set_dir(motor->in1_pin, GPIO_OUT);
  gpio_put(motor->in1_pin, 0);

  gpio_init(motor->in2_pin);
  gpio_set_dir(motor->in2_pin, GPIO_OUT);
  gpio_put(motor->in2_pin, 0);

  // PWM pin
  gpio_set_function(motor->pwm_pin, GPIO_FUNC_PWM);
  motor->pwm_slice = pwm_gpio_to_slice_num(motor->pwm_pin);
  motor->pwm_channel = pwm_gpio_to_channel(motor->pwm_pin);

  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, PWM_WRAP);
  pwm_config_set_clkdiv(&config, 1.0f);
  pwm_init(motor->pwm_slice, &config, true);
  pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, 0);
}

void motor_set(motor_t* motor, float velocity) {
  // Clamp to [-1.0, 1.0]
  if (velocity > 1.0f) velocity = 1.0f;
  if (velocity < -1.0f) velocity = -1.0f;

  if (fabsf(velocity) < 0.001f) {
    // Set level to 0 before disabling so the output pin is LOW when the counter stops.
    pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, 0);
    gpio_put(motor->in1_pin, 0);
    gpio_put(motor->in2_pin, 0);
  } else if (velocity > 0) {
    gpio_put(motor->in1_pin, 1);
    gpio_put(motor->in2_pin, 0);
    pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, (uint16_t)(velocity * PWM_WRAP));
    pwm_set_enabled(motor->pwm_slice, true);
  } else {
    gpio_put(motor->in1_pin, 0);
    gpio_put(motor->in2_pin, 1);
    pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, (uint16_t)(-velocity * PWM_WRAP));
    pwm_set_enabled(motor->pwm_slice, true);
  }
}
