#include "headers\i2c_master.h"
#include "driver/i2c.h"

// I2C Configuration for TSL2591
#define I2C_MASTER_SCL_IO GPIO_NUM_6    // SCL GPIO
#define I2C_MASTER_SDA_IO GPIO_NUM_5    // SDA GPIO
#define I2C_MASTER_NUM I2C_NUM_0        // I2C port number
#define I2C_MASTER_FREQ_HZ 100000       // I2C frequency
#define I2C_MASTER_TIMEOUT_MS 1000

// I2C initialization for TSL2591
esp_err_t i2c_master_init(void) {
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
