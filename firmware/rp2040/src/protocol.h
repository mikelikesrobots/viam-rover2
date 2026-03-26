#pragma once

#include "hardware/uart.h"
#include "motor.h"

/**
 * Bitmask result of protocol_poll indicating what was received.
 * Nothing received leaves result as 0.
 */
typedef enum {
  PROT_MOTOR_CMD_RX = 1,
} protocol_rx_result_t;

/**
 * Protocol handler state. Stores UART instance, motor pointers, and line buffer
 * for incoming command parsing.
 */
typedef struct {
  uart_inst_t* uart;
  motor_t* left_motor;
  motor_t* right_motor;
  char line_buf[64];
  int line_pos;
} protocol_t;

/**
 * Initialise protocol handler with data in protocol_t struct.
 * This should be called before protocol_poll.
 */
void protocol_init(protocol_t* proto);

/**
 * Poll protocol handler for incoming data.
 * Returns a bitmask indicating what was received.
 */
protocol_rx_result_t protocol_poll(protocol_t* proto);
