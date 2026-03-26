#include "protocol.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "encoder.h"
#include "telemetry.h"

/**
 * Send response with carriage return and newline.
 */
static void send_response(protocol_t* proto, const char* msg) {
  uart_puts(proto->uart, msg);
  uart_puts(proto->uart, "\r\n");
}

/**
 * Process a single line received from UART.
 */
static protocol_rx_result_t process_line(protocol_t* proto) {

  // Set final character to null terminator for easier parsing.
  proto->line_buf[proto->line_pos] = '\0';
  char* line = proto->line_buf;
  protocol_rx_result_t result = 0;

  if (line[0] == 'P') {
    // Handle ping command
    send_response(proto, "P");
  } else if (line[0] == 'S') {
    // Stop motors
    motor_set(proto->left_motor, 0.0f);
    motor_set(proto->right_motor, 0.0f);
    send_response(proto, "OK");
    result |= PROT_MOTOR_CMD_RX;
  } else if (line[0] == 'R') {
    // Reset encoder counts
    encoder_reset();
    send_response(proto, "OK");
  } else if (line[0] == 'E') {
    // Respond with current encoder counts
    char buf[32];
    snprintf(buf, sizeof(buf), "E %ld %ld", (long)encoder_get_left_ticks(),
             (long)encoder_get_right_ticks());
    send_response(proto, buf);
  } else if (line[0] == 'M') {
    // Set motor velocities: M <left_vel> <right_vel>
    char* p = line + 1;
    char* endp;
    float left_vel = strtof(p, &endp);
    if (endp == p) {
      send_response(proto, "ERR parse_fail");
      return result;
    }
    p = endp;
    float right_vel = strtof(p, &endp);
    if (endp == p) {
      send_response(proto, "ERR parse_fail");
      return result;
    }
    motor_set(proto->left_motor, left_vel);
    motor_set(proto->right_motor, right_vel);
    send_response(proto, "OK");
    result |= PROT_MOTOR_CMD_RX;
  } else if (strncmp(line, "TS ", 3) == 0) {
    // Set telemetry rate: TS <E|I|B> <hz>
    char type = 0;
    int hz = 0;
    if (sscanf(line + 3, "%c %d", &type, &hz) == 2) {
      telemetry_set_rate(type, (uint32_t)hz);
      send_response(proto, "OK");
    } else {
      send_response(proto, "ERR parse_fail");
    }
  } else if (strncmp(line, "TD ", 3) == 0) {
    // Disable telemetry for a stream: TD <E|I|B>
    char type = 0;
    if (sscanf(line + 3, "%c", &type) == 1) {
      telemetry_enable(type, false);
      send_response(proto, "OK");
    } else {
      send_response(proto, "ERR parse_fail");
    }
  } else {
    send_response(proto, "ERR bad_cmd");
  }

  return result;
}

void protocol_init(protocol_t* proto) {
  proto->line_pos = 0;
}

protocol_rx_result_t protocol_poll(protocol_t* proto) {
  protocol_rx_result_t result = 0;

  while (uart_is_readable(proto->uart)) {
    char ch = uart_getc(proto->uart);
    if (ch == '\n' || ch == '\r') {
      if (proto->line_pos > 0) {
        result |= process_line(proto);
        proto->line_pos = 0;
      }
    } else {
      if (proto->line_pos < (int)sizeof(proto->line_buf) - 1) {
        proto->line_buf[proto->line_pos++] = ch;
      }
    }
  }

  return result;
}
