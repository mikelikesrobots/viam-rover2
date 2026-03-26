#include "encoder.h"

#include "hardware/gpio.h"
#include "pico/time.h"

static volatile _Atomic int32_t left_ticks = 0;
static volatile _Atomic int32_t right_ticks = 0;
static uint32_t left_gpio = 0;
static uint32_t right_gpio = 0;

static void gpio_callback(uint gpio, uint32_t events) {
  if (gpio == left_gpio) {
    left_ticks++;
  } else if (gpio == right_gpio) {
    right_ticks++;
  }
}

void encoder_init(uint32_t left_pin, uint32_t right_pin) {
  left_gpio = left_pin;
  right_gpio = right_pin;

  gpio_init(left_pin);
  gpio_set_dir(left_pin, GPIO_IN);
  gpio_pull_up(left_pin);
  gpio_set_irq_enabled_with_callback(left_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

  gpio_init(right_pin);
  gpio_set_dir(right_pin, GPIO_IN);
  gpio_pull_up(right_pin);
  gpio_set_irq_enabled_with_callback(right_pin, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

int32_t encoder_get_left_ticks(void) { return left_ticks; }

int32_t encoder_get_right_ticks(void) { return right_ticks; }

void encoder_reset(void) {
  left_ticks = 0;
  right_ticks = 0;
}
