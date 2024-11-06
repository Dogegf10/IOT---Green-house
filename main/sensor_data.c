#include "headers\sensor_data.h"
#include "headers\dht11.h"
#include "headers\tsl2591.h"
#include "headers\mqtt.h"
#include "headers\rgb_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "esp_mac.h"

void read_and_publish_dht11_data(void) {
    int humidity = 0, temperature = 0;
    if (dht11_read_data(&humidity, &temperature) == 0) {
        printf("Temperature: %d°C, Humidity: %d%%\n", temperature, humidity);

        char temp_str[16], hum_str[16];
        sprintf(temp_str, "%d", temperature);
        sprintf(hum_str, "%d", humidity);

        blink_led_for_publish();  // Indicate sending data
        mqtt_publish_temperature(temp_str);
        mqtt_publish_humidity(hum_str);

        vTaskDelay(100 / portTICK_PERIOD_MS);  // Brief white blink
    } else {
        printf("Failed to read data from DHT11 sensor\n");
    }
}

void read_and_publish_tsl2591_data(void) {
    uint16_t lux = tsl2591_get_lux();
    printf("Light Level: %d lux\n", lux);

    char lux_str[16];
    sprintf(lux_str, "%d", lux);

    blink_led_for_publish();  // Indicate sending data
    mqtt_publish_light(lux_str);

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Brief white blink
}
