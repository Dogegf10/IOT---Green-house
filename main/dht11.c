#include "headers\dht11.h"
#include "driver\gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DHT11";
#define DHT11_PIN GPIO_NUM_3    // GPIO pin for DHT11

// Function to read data from the DHT11 sensor
int dht11_read_data(int *humidity, int *temperature) {
    gpio_set_direction(DHT11_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT11_PIN, 0);
    vTaskDelay(20 / portTICK_PERIOD_MS);  // Wait for 20ms
    gpio_set_level(DHT11_PIN, 1);
    esp_rom_delay_us(30);

    gpio_set_direction(DHT11_PIN, GPIO_MODE_INPUT);

    int timeout = 0;
    while (gpio_get_level(DHT11_PIN) == 1) {
        if (++timeout > 100) return -1;
        esp_rom_delay_us(1);
    }

    timeout = 0;
    while (gpio_get_level(DHT11_PIN) == 0) {
        if (++timeout > 100) return -1;
        esp_rom_delay_us(1);
    }

    timeout = 0;
    while (gpio_get_level(DHT11_PIN) == 1) {
        if (++timeout > 100) return -1;
        esp_rom_delay_us(1);
    }

    int data[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 40; i++) {
        timeout = 0;
        while (gpio_get_level(DHT11_PIN) == 0) {
            if (++timeout > 100) return -1;
            esp_rom_delay_us(1);
        }

        esp_rom_delay_us(40);
        if (gpio_get_level(DHT11_PIN) == 1) {
            data[i / 8] <<= 1;
            data[i / 8] |= 1;
        } else {
            data[i / 8] <<= 1;
        }

        timeout = 0;
        while (gpio_get_level(DHT11_PIN) == 1) {
            if (++timeout > 100) return -1;
            esp_rom_delay_us(1);
        }
    }

    if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        ESP_LOGE(TAG, "Checksum error");
        return -1;
    }

    *humidity = data[0];
    *temperature = data[2];
    return 0;
}


