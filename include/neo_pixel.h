#ifndef NEO_PIXEL_H
#define NEO_PIXEL_H

#include "led.h"

void neo_pixel_init(void);

void neo_pixel_render(led_data_t *leds);

int is_busy(void);

void pretty_delay(led_data_t *leds, int delay_ms);

#endif // NEO_PIXEL_H
