#include "headers\mqtt.h"
#include "esp_log.h"
#include "mqtt_client.h"

#define MQTT_BROKER_URI "mqtts://09e3753d24fe4de28e5cf18365cf7a9a.s1.eu.hivemq.cloud:8883" // Replace with your broker URI
#define MQTT_USERNAME "ESP32"  // If authentication is required
#define MQTT_PASSWORD "Nemkode123"  // If authentication is required
#define MQTT_TOPIC_TEMPERATURE "sensor/temperature"
#define MQTT_TOPIC_HUMIDITY "sensor/humidity"
#define MQTT_TOPIC_LIGHT "sensor/light"

static const char *TAG = "MQTT";
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

void mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = { .address = { .uri = MQTT_BROKER_URI } },
        .credentials = { .username = MQTT_USERNAME, .authentication = { .password = MQTT_PASSWORD } },
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

bool mqtt_start_client(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = { .address = { .uri = MQTT_BROKER_URI } },
        .credentials = { .username = MQTT_USERNAME, .authentication = { .password = MQTT_PASSWORD } },
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE("MQTT", "Failed to initialize MQTT client.");
        return false;
    }

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE("MQTT", "Failed to start MQTT client: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

void mqtt_publish_temperature(const char *temperature) {
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_TEMPERATURE, temperature, 0, 1, 0);
}

void mqtt_publish_humidity(const char *humidity) {
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_HUMIDITY, humidity, 0, 1, 0);
}

void mqtt_publish_light(const char *lux) {
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_LIGHT, lux, 0, 1, 0);
}