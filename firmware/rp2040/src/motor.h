#pragma once

#include <stdint.h>

typedef struct {
  uint32_t in1_pin;
  uint32_t in2_pin;
  uint32_t pwm_pin;
  uint32_t pwm_slice;
  uint32_t pwm_channel;
} motor_t;

/**
 * Initialise motor driver using information in motor_t struct.
 */
void motor_init(motor_t* motor);

/**
 * Set motor velocity in range [-1.0, 1.0]. Negative values indicate reverse direction.
 */
void motor_set(motor_t* motor, float velocity);
