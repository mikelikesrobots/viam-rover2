#pragma once

#include <stdint.h>

/**
 * Initialise encoder structure with left and right GPIO pin numbers.
 */
void encoder_init(uint32_t left_pin, uint32_t right_pin);

/**
 * Get number of ticks received from <side> encoder.
 */
int32_t encoder_get_left_ticks(void);
int32_t encoder_get_right_ticks(void);

/**
 * Reset all encoder tick counts to zero.
 */
void encoder_reset(void);
