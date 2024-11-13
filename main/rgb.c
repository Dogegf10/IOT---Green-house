#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_rom_sys.h"  // Required for esp_rom_delay_us()
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h" // To Hivemq cloud
#include "esp_crt_bundle.h" //to verification
#include "led_strip.h"


//RGB-LED
#define RGB_LED_PIN GPIO_NUM_8   // Replace this with your RGB LED GPIO pin
#define LED_STRIP_LENGTH 1       // Number of LEDs in the strip, usually 1 for a single RGB LED

led_strip_handle_t led_strip;


void rgb_led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
    led_strip_set_pixel(led_strip, 0, red, green, blue);  // Set color for first LED
    led_strip_refresh(led_strip);  // Refresh to apply the color
}

void rgb_led_init(void)
{
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_LED_PIN,
        .max_leds = LED_STRIP_LENGTH,  // Number of LEDs in the strip
    };

    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,  // 10MHz resolution
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);  // Ensure the LED is off initially
}
