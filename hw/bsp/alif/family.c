/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

/* metadata:
   manufacturer: Nordic Semiconductor
*/

#include "M55_HP.h"

#include "bsp/board_api.h"
#include "board.h"

void SysTick_Handler(void);
void USB_IRQHandler(void);

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+
void USB_IRQHandler(void)
{
    dcd_int_handler(0);
}

/*------------------------------------------------------------------*/
/* MACRO TYPEDEF CONSTANT ENUM
 *------------------------------------------------------------------*/

#define GPIO_OE_OFFSET        0x00  // Output Enable Register
#define GPIO_VAL_OFFSET       0x04  // Output Value Register

#define GPIO12_PORT                     12  /*< Use LED0_R,LED0_B GPIO port >*/
#define GPIO7_PORT                      7   /*< Use LED0_G GPIO port >*/
#define PIN3                            3   /*< LED0_R gpio pin >*/
#define PIN4                            4   /*< LED0_G gpio pin >*/
#define PIN0                            0   /*< LED0_B gpio pin >*/

/* LED1 gpio pins */
#define GPIO6_PORT                      6   /*< Use LED1_R,LED1_B,LED1_R GPIO port >*/
#define PIN2                            2   /*< LED1_R gpio pin >*/
#define PIN4                            4   /*< LED1_G gpio pin >*/
#define PIN6                            6   /*< LED1_B gpio pin >*/

#define REG32(addr)           (*(volatile uint32_t *)(addr))

#define USB_CTRL_BASE   0x48200000
#define USB_ALIF_IRQ    101

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+

static void pin_set_value(const uint8_t pin, const bool value)
{
    // Enable output for the pin
    REG32(GPIO0_BASE + GPIO_OE_OFFSET) |= (1U << pin);

    if (value) {
        // Set pin high
        REG32(GPIO0_BASE + GPIO_VAL_OFFSET) |= (1U << pin);
    } else {
        // Set pin low
        REG32(GPIO0_BASE + GPIO_VAL_OFFSET) &= ~(1U << pin);
    }
}

static void pin_set(const uint8_t port, const uint8_t pin, const uint8_t alt_func, const uint8_t pad_ctrl)
{
    uint32_t offset;
    offset = (uint32_t) ((port << 5) + (pin << 2));
    *((volatile uint32_t *) (PINMUX_BASE + offset)) = ((uint32_t) pad_ctrl << 16) | alt_func;
}

void board_init(void) {

    *(volatile uint32_t*) 0x4900C004 |= 8; // 12_0 + 12_3 as output (blue + red LED)
    *(volatile uint32_t*) 0x49007004 |= 16; // 7_4 as output (green LED)

    pin_set(GPIO12_PORT, PIN3, 0, 0);
    pin_set(GPIO7_PORT, PIN4, 0, 0);
    pin_set(GPIO12_PORT, PIN0, 0, 0);
    pin_set(GPIO6_PORT, PIN2, 0, 0);
    pin_set(GPIO6_PORT, PIN4, 0, 0);
    pin_set(GPIO6_PORT, PIN6, 0, 0);

    board_led_write(true);

#if CFG_TUSB_OS == OPT_OS_ZEPHYR
    IRQ_CONNECT(USB_ALIF_IRQ, 5, USB_IRQHandler, NULL, 0);
#endif
}

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+
void board_led_write(bool state) {
    (void) state;

    pin_set_value(PIN3, state);
    pin_set_value(PIN4, state);
    pin_set_value(PIN0, state);
    pin_set_value(PIN2, state);
    pin_set_value(PIN4, state);
    pin_set_value(PIN6, state);
}

uint32_t board_button_read(void) {
    return 1;
}

size_t board_get_unique_id(uint8_t id[], size_t max_len) {
  (void) max_len;
  (void) id;
  return 8;
}

int board_uart_read(uint8_t* buf, int len) {
  (void) buf;
  (void) len;
  return 0;
}

int board_uart_write(void const* buf, int len) {
  (void) buf;
  (void) len;
  return 1;
}

#if CFG_TUSB_OS == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void) {
  system_ticks++;
}

uint32_t board_millis(void) {
  return system_ticks;
}
#endif


int _close(int val);
int _lseek(int val0, int val1, int val2);
// Stubs to suppress missing stdio definitions for nosys
// #define TRAP_RET_ZERO  {__BKPT(0); return 0;}
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
//int _read(int val0, char * val1, int val2) TRAP_RET_ZERO
//int _write(int val0, char * val1, int val2) TRAP_RET_ZERO
// int _fstat(int val0, void * val1) TRAP_RET_ZERO
// int _isatty(int val0) TRAP_RET_ZERO
// int _getpid(void) TRAP_RET_ZERO
// void _kill(int val0, int val1) {__BKPT(0);}