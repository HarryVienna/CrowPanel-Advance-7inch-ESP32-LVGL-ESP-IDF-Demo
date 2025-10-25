#pragma once
#include <time.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define STC8H1K28_I2C_ADDR 0x30

typedef struct stc8h1k28_dev_t {
    i2c_master_dev_handle_t i2c_dev;
} stc8h1k28_dev_t;

// Typdefinition für das Geräte-Handle
typedef struct stc8h1k28_dev_t *stc8h1k28_handle_t;


esp_err_t stc8h1k28_init(i2c_master_bus_handle_t bus_handle, stc8h1k28_handle_t *stc8h1k28_handle, const i2c_device_config_t *config);

void stc8h1k28_deinit(stc8h1k28_handle_t dev);

esp_err_t stc8h1k28_set_brightness(stc8h1k28_handle_t dev, uint8_t brightness);

esp_err_t stc8h1k28_buzzer_on(stc8h1k28_handle_t dev);

esp_err_t stc8h1k28_buzzer_off(stc8h1k28_handle_t dev);

#ifdef __cplusplus
}
#endif