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
#include "rgb.c"
#include "driver/adc.h"

static const char *TAG = "Sensors";
#pragma region NVS

// Function to initialize NVS with error handling for "no free pages" or "new version found"
esp_err_t nvs_initialize(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW("NVS", "NVS flash needs to be erased, performing erase.");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("NVS", "NVS initialized successfully");
    return ret;
}

#pragma endregion

#pragma region MQTT

#define MQTT_BROKER_URI "mqtts://09e3753d24fe4de28e5cf18365cf7a9a.s1.eu.hivemq.cloud:8883" // Replace with your broker URI
#define MQTT_USERNAME "ESP32"  // If authentication is required
#define MQTT_PASSWORD "Nemkode123"  // If authentication is required

#define MQTT_TOPIC_TEMPERATURE "sensor/temperature"
#define MQTT_TOPIC_HUMIDITY "sensor/humidity"
#define MQTT_TOPIC_LIGHT "sensor/light"

static esp_mqtt_client_handle_t mqtt_client = NULL;
// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_ESP_TLS) {
                ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
        default:
            break;
    }
}

esp_mqtt_client_handle_t mqtt_init(esp_mqtt_client_handle_t mqtt_client1)
{
        // Initialize MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
            },
             .verification = {
                .crt_bundle_attach = esp_crt_bundle_attach,
            },
        },
        .credentials = {
            .username = MQTT_USERNAME,
            .authentication = {
                .password = MQTT_PASSWORD,
            },
        },
    };

    mqtt_client1 = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client1, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client1);
    return mqtt_client1;
}
#pragma  endregion

#pragma region TSL2591
// I2C Configuration for TSL2591
#define I2C_MASTER_SCL_IO GPIO_NUM_6    // SCL GPIO
#define I2C_MASTER_SDA_IO GPIO_NUM_5    // SDA GPIO
#define I2C_MASTER_NUM I2C_NUM_0        // I2C port number
#define I2C_MASTER_FREQ_HZ 100000       // I2C frequency
#define I2C_MASTER_TIMEOUT_MS 1000
#define TSL2591_ADDR 0x29               // TSL2591 I2C address

// I2C initialization for TSL2591
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Write to TSL2591
static esp_err_t tsl2591_write(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2591_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Read from TSL2591
static esp_err_t tsl2591_read(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2591_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start for reading
    i2c_master_write_byte(cmd, (TSL2591_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Initialize TSL2591 sensor
esp_err_t tsl2591_init(void) {
    esp_err_t err = tsl2591_write(0xA0, 0x03);  // Power on the sensor
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TSL2591");
    }
    return err;
}

// Read light level from TSL2591
uint16_t tsl2591_get_lux(void) {
    uint8_t data[2];
    esp_err_t err = tsl2591_read(0xB4, data, 2);  // Read light data
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from TSL2591");
        return 0;
    }
    return (data[1] << 8) | data[0];  // Combine two bytes into a 16-bit value
}
#pragma endregion

#pragma region DHT11
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


#pragma endregion

#pragma region WIFI
#define WIFI_SSID "Pavens"     //"FASTSPEED-2.4Ghz" 
#define WIFI_PASS "esp12345" //"A6DE8ED8"         


static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;


static const char *WIFI_TAG = "Wi-Fi";

// Wi-Fi event handler with detailed logs
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(WIFI_TAG, "Wi-Fi started. Attempting to connect...");
        esp_wifi_connect();
        // Set LED to red during connection attempt
        rgb_led_set_color(255, 0, 0);  // Red
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(WIFI_TAG, "Wi-Fi disconnected. Attempting to reconnect...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        // Set LED to red on Wi-Fi disconnection
        rgb_led_set_color(255, 0, 0);  // Red
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_str[16];  // Buffer to hold the IP address as a string
        ESP_LOGI(WIFI_TAG, "Got IP address: %s", esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str)));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        // Set LED to green once connected to Wi-Fi
        rgb_led_set_color(0, 255, 0);  // Green
    }
}

// Initialize Wi-Fi
void wifi_init(void) {
    ESP_LOGI(WIFI_TAG, "Initializing Wi-Fi...");
    
    esp_netif_init();
    wifi_event_group = xEventGroupCreate();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK, // Ensure WPA2 is used for the hotspot
            .scan_method = WIFI_FAST_SCAN,   // Faster scan method
            .channel = 1,                    // Specify a channel if known
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL  // Connect to the strongest AP
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(WIFI_TAG, "Wi-Fi initialization complete. Waiting for connection...");
}

// Function to wait for Wi-Fi connection and indicate status with LED
_Bool wifi_wait_for_connection(void) {
    // Wait for the connection to establish
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           portMAX_DELAY);  // Wait indefinitely

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "Connected to Wi-Fi.");
        rgb_led_set_color(0, 255, 0);  // Green - connected to Wi-Fi
        return true;
    } else {
        ESP_LOGE(WIFI_TAG, "Failed to connect to Wi-Fi.");
        rgb_led_set_color(255, 0, 0);  // Red - disconnected from Wi-Fi
        return false;  // Exit if no Wi-Fi connection
    }
}
// Function to reset LED based on Wi-Fi connection status
void check_wifi_and_reset_led(void) {
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);
    if (bits & WIFI_CONNECTED_BIT) {
        rgb_led_set_color(0, 255, 0);  // Green if connected to Wi-Fi
    } else {
        rgb_led_set_color(255, 0, 0);  // Red if not connected to Wi-Fi
    }
}
#pragma endregion

#pragma region MQTT Publish
// Function to read DHT11 data and publish it over MQTT
void read_and_publish_dht11_data(void) {
    int humidity = 0, temperature = 0;

    if (dht11_read_data(&humidity, &temperature) == 0) {
        printf("Temperature: %d°C, Humidity: %d%%\n", temperature, humidity);
        
        // Convert data to strings for MQTT
        char temp_str[16], hum_str[16];
        sprintf(temp_str, "%d", temperature);
        sprintf(hum_str, "%d", humidity);

        // Indicate data being sent by blinking the LED white
        rgb_led_set_color(255, 255, 255);  // White

        // Publish data to MQTT
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_TEMPERATURE, temp_str, 0, 1, 0);
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_HUMIDITY, hum_str, 0, 1, 0);

        // Short delay to indicate blink
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } else {
        printf("Failed to read data from DHT11 sensor\n");
    }
}

// Function to read TSL2591 data and publish it over MQTT
void read_and_publish_tsl2591_data(void) {
    int lux = tsl2591_get_lux();
    printf("Light Level: %d lux\n", lux);

    // Convert lux level to string for MQTT
    char lux_str[16];
    sprintf(lux_str, "%d", lux);

    // Indicate data being sent by blinking the LED white
    rgb_led_set_color(255, 255, 255);  // White

    // Publish lux level to MQTT
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_LIGHT, lux_str, 0, 1, 0);

    // Short delay to indicate blink
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
#pragma endregion

#pragma region PUMP
// Define your pins
#define PUMP_CONTROL_PIN GPIO_NUM_1      // Digital output for pump control
#define HUMIDITY_SENSOR_PIN ADC1_CHANNEL_0  // Analog input for humidity sensor
// Calibrate these values based on observed readings
#define DRY_READING 1841     // ADC reading when the soil is dry
#define WET_READING 1440     // ADC reading when the soil is wet
#define HUMIDITY_THRESHOLD 50 // Threshold percentage for turning on the pump

// Initialize GPIO and ADC
void pump_control_init() {
    // Configure pump control pin as output
    gpio_set_direction(PUMP_CONTROL_PIN, GPIO_MODE_OUTPUT);

    // Configure humidity sensor pin as ADC input (no additional setup needed for ADC)
    adc1_config_width(ADC_WIDTH_BIT_12);                  // 12-bit resolution
    adc1_config_channel_atten(HUMIDITY_SENSOR_PIN, ADC_ATTEN_DB_11);  // Set attenuation for full range
}

// Function to read humidity as a percentage
int read_humidity_percentage() {
    int raw_value = adc1_get_raw(HUMIDITY_SENSOR_PIN);
    int min_value = WET_READING;  // ADC reading in water
    int max_value = DRY_READING;  // ADC reading in air

    // Ensure raw_value is within min and max range
    if (raw_value < min_value) raw_value = min_value;
    if (raw_value > max_value) raw_value = max_value;

    // Invert the scaling to calculate humidity percentage
    int scaled_value = 100 - ((raw_value - min_value) * 100 / (max_value - min_value));
    return scaled_value;  // Returns humidity percentage
}


// Function to control the pump based on humidity
void control_pump_based_on_humidity() {
    int humidity_percentage = read_humidity_percentage();
    int raw_humidity_value = adc1_get_raw(HUMIDITY_SENSOR_PIN);
    char temp_soil_hum[16];

    // Print the humidity percentage
    sprintf(temp_soil_hum, "%d%%", humidity_percentage);
    printf("Raw ADC Value: %d, Humidity Percentage: %d%%\n", raw_humidity_value, humidity_percentage);

    if (humidity_percentage < HUMIDITY_THRESHOLD) {
        // Low humidity detected, turn on the pump
        gpio_set_level(PUMP_CONTROL_PIN, 1);  // Turn on the pump
        printf("Pump turned ON due to low humidity.\n");
    } else {
        // High humidity detected, turn off the pump
        gpio_set_level(PUMP_CONTROL_PIN, 0);  // Turn off the pump
        printf("Pump turned OFF due to sufficient humidity.\n");
    }
}
#pragma endregion


void app_main(void) {
    // Initialize NVS
    nvs_initialize();
    // Initialize RGB and Wifi
    rgb_led_init();
    wifi_init();
    wifi_wait_for_connection();
    
    // Initialize MQTT
    mqtt_client = mqtt_init(mqtt_client);

    // Initialize I2C for TSL2591
    i2c_master_init();
    tsl2591_init();
    pump_control_init();

   while (1) {
        read_and_publish_dht11_data();
        read_and_publish_tsl2591_data();
        control_pump_based_on_humidity();
        check_wifi_and_reset_led();
        vTaskDelay(60000 / portTICK_PERIOD_MS);  // Wait for 1 minute
    }
}
