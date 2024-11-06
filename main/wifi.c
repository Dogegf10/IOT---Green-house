#include "headers\wifi.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "headers\rgb_led.h"

static const char *WIFI_TAG = "Wi-Fi";
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// Wi-Fi credentials
#define WIFI_SSID "ESP32"     //"FASTSPEED-2.4Ghz" 
#define WIFI_PASS "Fisse6969" //"A6DE8ED8"  

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(WIFI_TAG, "Wi-Fi started. Attempting to connect...");
        esp_wifi_connect();
        rgb_led_set_color(255, 0, 0);  // Red
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(WIFI_TAG, "Wi-Fi disconnected. Attempting to reconnect...");
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        rgb_led_set_color(255, 0, 0);  // Red
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        char ip_str[16];
        ESP_LOGI(WIFI_TAG, "Got IP address: %s", esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str)));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        rgb_led_set_color(0, 255, 0);  // Green
    }
}

void wifi_init(void) {
    ESP_LOGI(WIFI_TAG, "Initializing Wi-Fi...");
    esp_netif_init();
    wifi_event_group = xEventGroupCreate();
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
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .scan_method = WIFI_FAST_SCAN,
            .channel = 1,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(WIFI_TAG, "Wi-Fi initialization complete. Waiting for connection...");
}

_Bool wifi_wait_for_connection(void) {
    // Wait for the connection to establish
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );  // Wait indefinitely

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(WIFI_TAG, "Connected to Wi-Fi.");
        rgb_led_set_color(0, 255, 0);  // Green - connected to Wi-Fi
    } else {
        ESP_LOGE(WIFI_TAG, "Failed to connect to Wi-Fi.");
        rgb_led_set_color(255, 0, 0);  // Red - disconnected from Wi-Fi
        return;  // Exit if no Wi-Fi connection
    }
}

