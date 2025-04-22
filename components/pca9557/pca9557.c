#include "pca9557.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <stdlib.h>

static const char *TAG = "PCA9557";


esp_err_t pca9557_init(i2c_master_bus_handle_t bus, pca9557_handle_t *expander_handle, const i2c_device_config_t *config)
{
    pca9557_dev_t *dev = malloc(sizeof(pca9557_dev_t));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Error allocating PCA9557 device handle");
        return ESP_FAIL;
    }

    esp_err_t err = i2c_master_bus_add_device(bus, config, &dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error adding PCA9557 to I2C bus: %s", esp_err_to_name(err));
        free(dev);
        return err;
    }
    ESP_LOGI(TAG, "PCA9557 device initialized with address 0x%02x", config->device_address);
    *expander_handle = (pca9557_handle_t)dev;

    return err;
}

esp_err_t pca9557_set_direction(pca9557_handle_t dev, uint8_t pin, uint8_t direction)
{
    if (pin > 7) {
        ESP_LOGE(TAG, "Invalid pin number: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }
    if (direction != PCA9557_INPUT && direction != PCA9557_OUTPUT) {
        ESP_LOGE(TAG, "Invalid direction: %d", direction);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config_reg_value;
    uint8_t reg_address = PCA9557_CONFIG_REG;
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg_address, 1, &config_reg_value, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading configuration register: %s", esp_err_to_name(err));
        return err;
    }

    if (direction == PCA9557_INPUT) {
        config_reg_value |= (1 << pin);
    } else {
        config_reg_value &= ~(1 << pin);
    }

    uint8_t data[2] = {PCA9557_CONFIG_REG, config_reg_value};
    err = i2c_master_transmit(dev->i2c_dev, data, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing configuration register: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGD(TAG, "Pin %d set as %s", pin, (direction == PCA9557_INPUT) ? "Input" : "Output");
    return ESP_OK;
}

esp_err_t pca9557_read_pin(pca9557_handle_t dev, uint8_t pin, int *value)
{
    if (pin > 7) {
        ESP_LOGE(TAG, "Invalid pin number: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }
    if (value == NULL) {
        ESP_LOGE(TAG, "Invalid pointer for value");
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t input_port_value;
    uint8_t reg_address = PCA9557_INPUT_PORT_REG;
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg_address, 1, &input_port_value, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading input port register: %s", esp_err_to_name(err));
        return err;
    }

    *value = (input_port_value >> pin) & 0x01;
    ESP_LOGD(TAG, "Pin %d read, value: %d", pin, *value);
    return ESP_OK;
}

esp_err_t pca9557_write_pin(pca9557_handle_t dev, uint8_t pin, int value)
{
    if (pin > 7) {
        ESP_LOGE(TAG, "Invalid pin number: %d", pin);
        return ESP_ERR_INVALID_ARG;
    }
    if (value != 0 && value != 1) {
        ESP_LOGE(TAG, "Invalid value: %d (expected 0 or 1)", value);
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t output_port_value;
    uint8_t reg_address = PCA9557_OUTPUT_PORT_REG;
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg_address, 1, &output_port_value, 1, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading output port register: %s", esp_err_to_name(err));
        return err;
    }

    if (value) {
        output_port_value |= (1 << pin);
    } else {
        output_port_value &= ~(1 << pin);
    }

    uint8_t data[2] = {PCA9557_OUTPUT_PORT_REG, output_port_value};
    err = i2c_master_transmit(dev->i2c_dev, data, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing output port register: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGD(TAG, "Pin %d set to %d", pin, value);
    return ESP_OK;
}

esp_err_t pca9557_delete(pca9557_handle_t dev)
{
    if (dev == NULL) {
        return ESP_OK; // Nothing to do
    }
    esp_err_t err = i2c_master_bus_rm_device(dev->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error removing I2C device: %s", esp_err_to_name(err));
        // Continue cleanup even if removal fails
    }
    free(dev);
    ESP_LOGI(TAG, "PCA9557 device deleted");
    // Return the error from i2c_master_bus_rm_device if it occurred
    return err;
}