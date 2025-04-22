#pragma once

#include <esp_err.h>
#include <driver/i2c_master.h>

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C address of the PCA9557
#define PCA9557_I2C_ADDRESS 0x18

// Register Addresses
#define PCA9557_INPUT_PORT_REG    0x00
#define PCA9557_OUTPUT_PORT_REG   0x01
#define PCA9557_POLARITY_INV_REG  0x02
#define PCA9557_CONFIG_REG        0x03

// Pin Directions
#define PCA9557_INPUT  1
#define PCA9557_OUTPUT 0

typedef struct pca9557_dev_t {
    i2c_master_dev_handle_t i2c_dev;
} pca9557_dev_t;

// Type definition for the device handle
typedef struct pca9557_dev_t *pca9557_handle_t;

/**
 * @brief Initializes the PCA9557 driver.
 *
 * @param bus The I2C bus handle.
 * @param expander_handle Pointer to store the created device handle.
 * @param config Pointer to the I2C device configuration structure (contains device address etc.).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9557_init(i2c_master_bus_handle_t bus, pca9557_handle_t *expander_handle, const i2c_device_config_t *config);

/**
 * @brief Sets the direction of a single pin.
 *
 * @param dev The device handle.
 * @param pin The pin number (0-7).
 * @param direction The direction (PCA9557_INPUT or PCA9557_OUTPUT).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9557_set_direction(pca9557_handle_t dev, uint8_t pin, uint8_t direction);

/**
 * @brief Reads the state of a single pin.
 *
 * @param dev The device handle.
 * @param pin The pin number (0-7).
 * @param value A pointer to store the read state (0 or 1).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9557_read_pin(pca9557_handle_t dev, uint8_t pin, int *value);

/**
 * @brief Writes a state to a single pin.
 *
 * @param dev The device handle.
 * @param pin The pin number (0-7).
 * @param value The state to write (0 or 1).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9557_write_pin(pca9557_handle_t dev, uint8_t pin, int value);

/**
 * @brief Releases the resources of the PCA9557 driver.
 *
 * @param dev The device handle.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t pca9557_delete(pca9557_handle_t dev); // Note: Functionality likely handled by i2c_master_bus_rm_device

#ifdef __cplusplus
}
#endif