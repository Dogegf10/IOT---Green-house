#include "headers\tsl2591.h"
#include "headers\i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "TSL2591";
#define TSL2591_ADDR 0x29


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