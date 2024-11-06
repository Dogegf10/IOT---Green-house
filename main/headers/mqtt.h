#ifndef MQTT_H
#define MQTT_H
#include <stdbool.h>

void mqtt_init(void);
bool mqtt_start_client(void);
void mqtt_publish_temperature(const char *temperature);
void mqtt_publish_humidity(const char *humidity);
void mqtt_publish_light(const char *lux);

#endif
