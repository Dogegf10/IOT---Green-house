#ifndef TSL2591_H
#define TSL2591_H
#include "esp_err.h"

esp_err_t tsl2591_init(void);
esp_err_t tsl2591_write(uint8_t reg, uint8_t value);
esp_err_t tsl2591_read(uint8_t reg, uint8_t *data, size_t len);
uint16_t tsl2591_get_lux(void);

#endif
