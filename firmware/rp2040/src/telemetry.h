#pragma once

#include "hardware/uart.h"
#include "pico/stdlib.h"

/**
 * Telemetry module for sending encoder, IMU, and battery data to host over UART.
 * The telemetry protocol consists of binary frames with the following format:
 * STX(0x02) LEN(1) PAYLOAD(LEN) CRC(1)
 * where CRC is a simple XOR of all payload bytes.
 */
void telemetry_init(uart_inst_t* uart);

/**
 * Poll telemetry module to send data at configured rates. This should be called
 * frequently in the main loop.
 */
void telemetry_poll(void);

/**
 * Enable or disable telemetry stream.
 * Stream types: 'E' = encoder, 'I' = IMU, 'B' = battery.
 */
void telemetry_enable(char stream, bool enabled);

/**
 * Set telemetry stream rate in Hz. A rate of 0 will disable the stream.
 * Stream types: 'E' = encoder, 'I' = IMU, 'B' = battery
 */
void telemetry_set_rate(char stream, uint32_t hz);
