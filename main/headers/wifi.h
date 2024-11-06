#ifndef WIFI_H
#define WIFI_H

#include "esp_event.h"

// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;


static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
void wifi_init(void);
_Bool wifi_wait_for_connection(void);

#endif
