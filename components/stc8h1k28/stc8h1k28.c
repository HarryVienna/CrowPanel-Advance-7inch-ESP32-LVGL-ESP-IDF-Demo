#include "stc8h1k28.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STC8H1K28";


esp_err_t stc8h1k28_init(i2c_master_bus_handle_t bus_handle, stc8h1k28_handle_t *extender_handle, const i2c_device_config_t *config) {
    ESP_RETURN_ON_FALSE(bus_handle && extender_handle && config, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    stc8h1k28_dev_t *dev = calloc(1, sizeof(stc8h1k28_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Memory allocation failed");

    // Initialize I2C device
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(bus_handle, config, &dev->i2c_dev),
        TAG, "Failed to add I2C device");

    *extender_handle = dev;
    ESP_LOGI(TAG, "stc8h1k28 initialized successfully");
    return ESP_OK;
}

void stc8h1k28_deinit(stc8h1k28_handle_t dev) {
    if (dev) {
        if (dev->i2c_dev) {
            i2c_master_bus_rm_device(dev->i2c_dev);
        }
        free(dev);
    }
}

esp_err_t stc8h1k28_set_brightness(stc8h1k28_handle_t dev, uint8_t brightness) {

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, &brightness, 1, -1),
        TAG, "Failed to set time");
    return ESP_OK;
}

esp_err_t stc8h1k28_buzzer_on(stc8h1k28_handle_t dev) {

    uint8_t value = 246;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, &value, 1, -1),
        TAG, "Failed to set time");
    return ESP_OK;
}

esp_err_t stc8h1k28_buzzer_off(stc8h1k28_handle_t dev) {

    uint8_t value = 247;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, &value, 1, -1),
        TAG, "Failed to set time");
    return ESP_OK;
}
