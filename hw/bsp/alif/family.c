#include "bsp/board_api.h"

//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_init(void)
{
    // Initialize board hardware (dummy)
}

void board_led_write(bool state)
{
    // Control onboard LED (dummy)
    (void) state;
}

uint32_t board_button_read(void)
{
    // Return button state (dummy)
    return 0;
}

size_t board_get_unique_id(uint8_t id[], size_t max_len)
{
    // Return dummy unique ID
    (void) max_len;

    for (size_t i = 0; i < 8; i++) {
        id[i] = i;
    }

    return 8;
}

uint32_t board_millis(void)
{
    // Dummy implementation
    return 0;
}
