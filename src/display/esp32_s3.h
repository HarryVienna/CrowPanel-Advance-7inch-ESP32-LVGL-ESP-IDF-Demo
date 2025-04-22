#ifndef ESP32_S3_H
#define ESP32_S3_H

#include <stdio.h>

#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "pca9557.h"
#include "bm8563.h"
#include "buzzer.h"

#ifdef __cplusplus
extern "C" {
#endif



// Function declarations
void init_display(void);

void init_buzzer(void);

void init_i2c(i2c_master_bus_handle_t *i2c_bus_handle);

void init_i2c_expander(i2c_master_bus_handle_t i2c_bus_handle, pca9557_handle_t *expander_handle);

void init_rtc(i2c_master_bus_handle_t i2c_bus_handle, bm8563_handle_t *rtc_handle);

void set_backlight_brightness(uint8_t brightness);

void init_touch(i2c_master_bus_handle_t i2c_bus_handle, pca9557_handle_t expander_handl, esp_lcd_touch_handle_t *touch_handlee);

void init_lcd(esp_lcd_panel_handle_t *panel_handle);

void init_backlight(pca9557_handle_t expander_handle);

void init_lvgl(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t touch_handle);

void set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);

esp_err_t get_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_S3_H */