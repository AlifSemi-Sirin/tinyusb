#include "bsp/board_api.h"
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

/**
 * @brief Board init: configure LED and button pins
 */
void board_init(void) {
  if (device_is_ready(led.port)) {
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
  }
  if (device_is_ready(button.port)) {
    gpio_pin_configure_dt(&button, GPIO_INPUT);
  }
}

/**
 * @brief Control board LED
 */
void board_led_write(bool state) {
  if (device_is_ready(led.port)) {
    gpio_pin_set(led.port, led.pin, state ? 1 : 0);
  }
}

/**
 * @brief Read button state (returns 1 if pressed, 0 otherwise)
 */
uint32_t board_button_read(void) {
  if (!device_is_ready(button.port)) {
    return 0;
  }
  int val = gpio_pin_get(button.port, button.pin);

  return (button.dt_flags & GPIO_ACTIVE_LOW) ? (val == 0) : (val != 0);
}

/**
 * @brief Get a unique ID for the board
 */
size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  size_t to_read = MIN(max_len, 8U);

#if DT_NODE_HAS_STATUS(DT_NODELABEL(entropy), okay)
  const struct device *ent = DEVICE_DT_GET(DT_NODELABEL(entropy));
  if (device_is_ready(ent)) {
    /* read from entropy device */
    entropy_get_entropy(ent, id, to_read);
    return to_read;
  }
#endif

  /* fallback: just fill with 0..n */
  for (size_t i = 0; i < to_read; i++) {
    id[i] = (uint8_t) i;
  }
  return to_read;
}

/**
 * @brief Returns the current time in milliseconds since boot
 */
uint32_t board_millis(void) {
  return k_uptime_get_32();
}

void USBD_IRQHandler(void)
{
  tud_int_handler(0);
}
