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
#include "headers\dht11.h"
#include "headers\rgb_led.h"
#include "headers\wifi.h"
#include "headers\mqtt.h"
#include "headers\i2c_master.h"
#include "headers\tsl2591.h"
#include "headers\sensor_data.h"

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize RGB LED, Wi-Fi, MQTT, I2C, and TSL2591
    rgb_led_init();
    wifi_init();
    mqtt_init();
    i2c_master_init();
    tsl2591_init();

    // Wait for Wi-Fi connection
    if (!wifi_wait_for_connection()) {
        return;  // Exit if no Wi-Fi connection
    }

     // Initialize and start MQTT client
    if (!mqtt_start_client()) {
        ESP_LOGE("Main", "MQTT client failed to start.");
        return;  // Exit if MQTT fails to start
    }

   while (1) {

        read_and_publish_dht11_data();
        read_and_publish_tsl2591_data();
        
        wifi_wait_for_connection();

        vTaskDelay(60000 / portTICK_PERIOD_MS);
    }
}
