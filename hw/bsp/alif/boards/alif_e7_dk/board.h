#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// Board initialization
void board_init(void);

// LED control
void board_led_write(bool state);

// Button read
uint32_t board_button_read(void);

// Unique ID read
size_t board_get_unique_id(uint8_t id[], size_t max_len);

// Millisecond counter
uint32_t board_millis(void);

#ifdef __cplusplus
}
#endif

#endif /* _BOARD_H_ */
