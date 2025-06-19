#include "bsp/board_api.h"
#include "board.h"
 
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  #include "RTE_Components.h"
  #include CMSIS_device_header

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
  // TODO: API implementation
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
  // TODO: implement API
  (void) state;
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
  // TODO: implement API
  return 1; 
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
 * @brief Unique ID of the board
 */
size_t board_get_unique_id(uint8_t id[], size_t max_len) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  // TODO: implement API
  (void) max_len;
  (void) id;
  return 0;
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR
  // TODO: implement API
#else 
  #error NOT Implemented!
#endif  
}

/**
 * @brief UART read handler
 */
int board_uart_read(uint8_t* buf, int len) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  // TODO: implement API
  (void) buf;
  (void) len;
  return 0;
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR
  // TODO: implement API
#else 
  #error NOT Implemented!
#endif  
}

int board_uart_write(void const* buf, int len) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
  // TODO: implement API
  (void) buf;
  (void) len;
  return 1;
#elif CFG_TUSB_OS == OPT_OS_ZEPHYR
  // TODO: implement API
#else 
  #error NOT Implemented!
#endif  
}

/**
 * @brief Returns the current time in milliseconds since boot
 */
#if CFG_TUSB_OS == OPT_OS_NONE

void SysTick_Handler(void);

volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}
#endif

#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS

void USB_IRQHandler(void);

void USB_IRQHandler(void)
{
    dcd_int_handler(0);
}

int _close(int val);
int _lseek(int val0, int val1, int val2);

int _close(int val) {
    (void) val;
    __BKPT(0); 
    return 0;
}
int _lseek(int val0, int val1, int val2) {
    (void) val0;
    (void) val1;
    (void) val2;
    __BKPT(0); 
    return 0;
}

#endif


#if CFG_TUSB_OS == OPT_OS_ZEPHYR

void USBD_IRQHandler(void)
{
  tud_int_handler(0);
}

#endif
