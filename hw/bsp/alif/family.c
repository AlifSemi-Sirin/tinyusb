#include "bsp/board_api.h"
#include "board.h"
#include <stdbool.h>

#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
#include "RTE_Components.h"
#include CMSIS_device_header
#include "Driver_GPIO.h"
#include "uart_tracelib.h"
#endif

#if CFG_TUSB_OS == OPT_OS_ZEPHYR
#include <zephyr/devicetree.h>
#include <zephyr/drivers/entropy.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static const struct gpio_dt_spec btn2 = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);
static const struct gpio_dt_spec btn3 = GPIO_DT_SPEC_GET(DT_ALIAS(sw3), gpios);
static const struct gpio_dt_spec btn4 = GPIO_DT_SPEC_GET(DT_ALIAS(sw4), gpios);
#endif

/**
 * @brief Board init: configure LED and button pins
 */
void board_init(void) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
      BOARD_Pinmux_Init();
    BOARD_BUTTON2_Init(NULL);

    // 1ms tick timer
    SysTick_Config(SystemCoreClock / 1000);
    
    tracelib_init(NULL, NULL);
#endif

#if CFG_TUSB_OS == OPT_OS_ZEPHYR
    if (device_is_ready(led.port)) {
      gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    }
    
    gpio_pin_configure_dt(&btn4, GPIO_INPUT | GPIO_PULL_UP);

      if (device_is_ready(btn0.port)) {
    //   gpio_pin_configure_dt(&btn0, GPIO_INPUT);
    //   gpio_pin_configure_dt(&btn1, GPIO_INPUT);
    //   gpio_pin_configure_dt(&btn2, GPIO_INPUT);
    //   gpio_pin_configure_dt(&btn3, GPIO_INPUT);
    //   gpio_pin_configure_dt(&btn4, GPIO_OUTPUT_INACTIVE);
    }
#endif
}

/**
 * @brief Control board LED
 */
void board_led_write(bool state) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
    BOARD_LED1_Control(state ?
        BOARD_LED_STATE_HIGH :
        BOARD_LED_STATE_LOW);
#endif
#if CFG_TUSB_OS == OPT_OS_ZEPHYR
    if (device_is_ready(led.port)) {
        gpio_pin_set(led.port, led.pin, state ? 1 : 0);
    }
#endif  
}

/**
 * @brief Read button state (returns 1 if pressed, 0 otherwise)
 */
uint32_t board_button_read(void) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
    BOARD_BUTTON_STATE btn_state;
    // Get new button state (active low)
    BOARD_BUTTON2_GetState(&btn_state);
    return BOARD_BUTTON_STATE_LOW == btn_state;
#endif
#if CFG_TUSB_OS == OPT_OS_ZEPHYR
    // if (!device_is_ready(btn0.port)) {
    //     return 0;
    // }
    static uint8_t cnt = 0;
    gpio_pin_set(led.port, led.pin, cnt & 1);
    gpio_pin_set(btn4.port, btn4.pin, 1);
    cnt++;
    // int val = gpio_pin_get(btn4.port, btn4.pin);


    // return (button.dt_flags & GPIO_ACTIVE_LOW) ? (val == 0) : (val != 0);
    // return val == 0;
    return 0;
#endif  
}

/**
 * @brief UART read handler
 */
int board_uart_read(uint8_t* buf, int len) {
    // NOTE: stdin functionality has not been implemented
    (void) buf, (void) len;
    return 0;
}

int board_uart_write(void const* buf, int len) {
#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
    int ret = send_str((const char *) buf, len);
    return (ret == ARM_DRIVER_OK) ? len : 0;
#endif

#if CFG_TUSB_OS == OPT_OS_ZEPHYR
    // TODO: implement API
    (void) buf, (void) len;
    return 0;
#endif  
}

#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS

struct stat;
int _fstat(int f, struct stat *buf);
int _isatty(int fh);
int _getpid(void);
int _kill(int pid, int sig);

// Stubs to suppress missing stdio definitions
int _close(int fh) {
    (void) fh;
    return -1;
}

int _lseek(int fh, long pos, int whence) {
    (void) fh, (void) pos, (void) whence;
    return -1;
}

struct stat;
int _fstat(int f, struct stat *buf) {
    (void) f, (void) buf;
    return -1;
}

int _isatty(int fh) {
    (void) fh;
    return 0;
}

int _getpid(void) {
    return 1;
}

int _kill(int pid, int sig) {
    (void) sig;

    if (pid == 1) {
        __BKPT(0);
        while(1) {
            __WFE();
        }
    }

    return -1;
}

#endif

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

/**
 * @brief Returns the current time in milliseconds since boot
 */
uint32_t board_millis(void) {
  return system_ticks;
}
#endif


#if CFG_TUSB_OS == OPT_OS_NONE || CFG_TUSB_OS == OPT_OS_FREERTOS
void USB_IRQHandler(void) {
    dcd_int_handler(0);
}
#endif

#if CFG_TUSB_OS == OPT_OS_ZEPHYR
void USBD_IRQHandler(void) {
    tud_int_handler(0);
}
#endif
