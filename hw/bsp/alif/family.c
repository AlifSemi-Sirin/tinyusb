#include "bsp/board_api.h"
 
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  #error TODO: Implement
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR

#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
#else 
  #error NOT Implemented!
#endif

/**
 * @brief Board init: configure LED and button pins
 */
void board_init(void) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  #error TODO: Implement    
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR
  if (device_is_ready(led.port)) {
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
  }
  if (device_is_ready(button.port)) {
    gpio_pin_configure_dt(&button, GPIO_INPUT);
  }
#else 
  #error NOT Implemented!
#endif  
}

/**
 * @brief Control board LED
 */
void board_led_write(bool state) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  #error TODO: Implement    
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR
  if (device_is_ready(led.port)) {
    gpio_pin_set(led.port, led.pin, state ? 1 : 0);
  }
#else 
  #error NOT Implemented!
#endif  
}

/**
 * @brief Read button state (returns 1 if pressed, 0 otherwise)
 */
uint32_t board_button_read(void) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  #error TODO: Implement    
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR
  if (!device_is_ready(button.port)) {
    return 0;
  }
  int val = gpio_pin_get(button.port, button.pin);

  return (button.dt_flags & GPIO_ACTIVE_LOW) ? (val == 0) : (val != 0);
#else 
  #error NOT Implemented!
#endif  
}

/**
 * @brief Returns the current time in milliseconds since boot
 */
#if CFG_TUSB_OS == OPT_OS_NONE
 uint32_t board_millis(void) {
  // ToDo: implement for baremetal
  return 0;
}
#endif

void USBD_IRQHandler(void)
{
  tud_int_handler(0);
}
