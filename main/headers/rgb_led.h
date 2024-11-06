#ifndef RGB_LED_H
#define RGB_LED_H
#include "esp_err.h"

void rgb_led_init(void);
void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue);
void blink_led_for_publish(void);

#endif
