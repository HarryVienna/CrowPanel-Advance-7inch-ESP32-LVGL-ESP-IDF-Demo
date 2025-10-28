#ifndef ESP32_S3_H
#define ESP32_S3_H

#include <stdio.h>
#include <time.h>

#include "esp_lcd_touch_gt911.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_timer.h"
#include "stc8h1k28.h"
#include "bm8563.h"

#ifdef __cplusplus
extern "C" {
#endif



// Function declarations
void init_display(void);

void init_i2c(i2c_master_bus_handle_t *i2c_bus_handle);

void init_extender(i2c_master_bus_handle_t i2c_bus_handle, stc8h1k28_handle_t *stc8h1k28_handle);

void init_rtc(i2c_master_bus_handle_t i2c_bus_handle, bm8563_handle_t *rtc_handle);

void init_touch(i2c_master_bus_handle_t i2c_bus_handle, esp_lcd_touch_handle_t *touch_handlee);

void init_lcd(esp_lcd_panel_handle_t *panel_handle);

void init_lvgl(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t touch_handle);

bool lvgl_port_lock(int timeout_ms);

void lvgl_port_unlock(void);

void set_backlight_brightness(uint8_t brightness);

void beep(uint16_t duration);

void set_time(uint8_t hours, uint8_t minutes, uint8_t seconds);

esp_err_t get_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds);

#ifdef __cplusplus
}
#endif

#endif /* ESP32_S3_H */