#include "bm8563.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BM8563";

// Register addresses
#define BM8563_REG_CTRL1      0x00
#define BM8563_REG_CTRL2      0x01
#define BM8563_REG_SECONDS    0x02
#define BM8563_REG_MINUTES    0x03
#define BM8563_REG_HOURS      0x04
#define BM8563_REG_DAYS       0x05
#define BM8563_REG_WEEKDAYS   0x06
#define BM8563_REG_MONTHS     0x07
#define BM8563_REG_YEARS      0x08
#define BM8563_REG_ALARM_MIN  0x09
#define BM8563_REG_ALARM_HOUR 0x0A
#define BM8563_REG_ALARM_DAY  0x0B
#define BM8563_REG_ALARM_WDAY 0x0C
#define BM8563_REG_CLKOUT     0x0D
#define BM8563_REG_TIMER_CTRL 0x0E
#define BM8563_REG_TIMER      0x0F

#define BM8563_I2C_TIMEOUT_MS 100  

// Helper functions
static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

esp_err_t bm8563_init(i2c_master_bus_handle_t bus_handle, bm8563_handle_t *rtc_handle, const i2c_device_config_t *config, gpio_num_t int_pin) {
    ESP_RETURN_ON_FALSE(bus_handle && rtc_handle && config, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    bm8563_dev_t *dev = calloc(1, sizeof(bm8563_dev_t));
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_NO_MEM, TAG, "Memory allocation failed");

    // Initialize I2C device
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(bus_handle, config, &dev->i2c_dev),
        TAG, "Failed to add I2C device");

    dev->int_pin = int_pin;
    
    // Initialize INT pin if configured
    if (int_pin >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << int_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure INT pin");
    }

    // Initialize RTC registers
    uint8_t init_data[2] = {BM8563_REG_CTRL1, 0x00}; // Stop clock, clear flags
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, init_data, sizeof(init_data), -1),
        TAG, "Failed to initialize RTC");

    *rtc_handle = dev;
    ESP_LOGI(TAG, "BM8563 initialized successfully");
    return ESP_OK;
}

void bm8563_deinit(bm8563_handle_t dev) {
    if (dev) {
        if (dev->i2c_dev) {
            i2c_master_bus_rm_device(dev->i2c_dev);
        }
        free(dev);
    }
}

esp_err_t bm8563_set_time(bm8563_handle_t dev, const struct tm *time) {
    ESP_RETURN_ON_FALSE(dev && time, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    // Validate time structure
    ESP_RETURN_ON_FALSE(time->tm_sec >= 0 && time->tm_sec <= 59, ESP_ERR_INVALID_ARG, TAG, "Invalid seconds");
    ESP_RETURN_ON_FALSE(time->tm_min >= 0 && time->tm_min <= 59, ESP_ERR_INVALID_ARG, TAG, "Invalid minutes");
    ESP_RETURN_ON_FALSE(time->tm_hour >= 0 && time->tm_hour <= 23, ESP_ERR_INVALID_ARG, TAG, "Invalid hours");
    ESP_RETURN_ON_FALSE(time->tm_mday >= 1 && time->tm_mday <= 31, ESP_ERR_INVALID_ARG, TAG, "Invalid day");
    ESP_RETURN_ON_FALSE(time->tm_mon >= 0 && time->tm_mon <= 11, ESP_ERR_INVALID_ARG, TAG, "Invalid month");
    ESP_RETURN_ON_FALSE(time->tm_year >= 0, ESP_ERR_INVALID_ARG, TAG, "Invalid year");

    uint8_t data[8] = {
        BM8563_REG_SECONDS,
        dec_to_bcd(time->tm_sec),
        dec_to_bcd(time->tm_min),
        dec_to_bcd(time->tm_hour),
        dec_to_bcd(time->tm_mday),
        dec_to_bcd(time->tm_wday % 7),
        dec_to_bcd(time->tm_mon + 1),
        dec_to_bcd(time->tm_year % 100)
    };

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, data, sizeof(data), -1),
        TAG, "Failed to set time");
    return ESP_OK;
}


esp_err_t bm8563_get_time(bm8563_handle_t dev, struct tm *time) {
    ESP_RETURN_ON_FALSE(dev && time, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    uint8_t reg = BM8563_REG_SECONDS;
    uint8_t data[7];
    
    // ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->i2c_dev, &reg, 1, BM8563_I2C_TIMEOUT_MS),
    //      TAG, "Failed to write i2c");
    // ESP_RETURN_ON_ERROR(i2c_master_receive(dev->i2c_dev, data, 7, BM8563_I2C_TIMEOUT_MS),
    //     TAG, "Failed to read i2c");

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, sizeof(data), BM8563_I2C_TIMEOUT_MS),
        TAG, "Failed to read time");

    //i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, sizeof(data), BM8563_I2C_TIMEOUT_MS);


    if ((data[0] & 0x80) ||  // VL-Bit (Clock integrity)
        (data[2] & 0x80)) {  // Invalid 24-hour format
        ESP_LOGE(TAG, "RTC data integrity error");
        //return ESP_ERR_INVALID_RESPONSE;
    }
    
    memset(time, 0, sizeof(struct tm));
    time->tm_sec = bcd_to_dec(data[0] & 0x7F);
    time->tm_min = bcd_to_dec(data[1] & 0x7F);
    time->tm_hour = bcd_to_dec(data[2] & 0x3F);
    time->tm_mday = bcd_to_dec(data[3] & 0x3F);
    time->tm_wday = bcd_to_dec(data[4] & 0x07);
    time->tm_mon = bcd_to_dec(data[5] & 0x1F) - 1;
    time->tm_year = bcd_to_dec(data[6]) + 100;

    return ESP_OK;
}

esp_err_t bm8563_set_alarm(bm8563_handle_t dev, const bm8563_alarm_t *alarm, bm8563_alarm_mode_t mode) {
    ESP_RETURN_ON_FALSE(dev && alarm, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
    uint8_t data[5] = {BM8563_REG_ALARM_MIN, 0x80, 0x80, 0x80, 0x80};
    
    if (alarm->active) {
        data[1] = dec_to_bcd(alarm->minutes);
        data[2] = dec_to_bcd(alarm->hours);
        
        switch (mode) {
            case BM8563_ALARM_WEEKLY:
                data[4] = dec_to_bcd(alarm->weekday) | 0x80;
                break;
            case BM8563_ALARM_ONETIME:
                data[3] = dec_to_bcd(alarm->day) | 0x80;
                break;
            case BM8563_ALARM_DAILY:
                break;
            default:
                break;
        }
    }

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, data, sizeof(data), -1),
        TAG, "Failed to set alarm");

    // Update control register
    uint8_t ctrl2;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, 
                                  (uint8_t[]){BM8563_REG_CTRL2}, 1, 
                                  &ctrl2, 1, -1),
        TAG, "Failed to read CTRL2");

    ctrl2 &= ~0x02;
    if (alarm->active && mode != BM8563_ALARM_DISABLED) {
        ctrl2 |= 0x02;
        dev->alarm_enabled = true;
    } else {
        dev->alarm_enabled = false;
    }

    uint8_t ctrl2_data[2] = {BM8563_REG_CTRL2, ctrl2};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, ctrl2_data, sizeof(ctrl2_data), -1),
        TAG, "Failed to update CTRL2");

    return ESP_OK;
}

esp_err_t bm8563_get_alarm(bm8563_handle_t dev, bm8563_alarm_t *alarm, bm8563_alarm_mode_t *mode) {
    ESP_RETURN_ON_FALSE(dev && alarm && mode, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t reg = BM8563_REG_ALARM_MIN;
    uint8_t data[4];
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, data, sizeof(data), -1),
        TAG, "Failed to read alarm");

    alarm->minutes = bcd_to_dec(data[0] & 0x7F);
    alarm->hours = bcd_to_dec(data[1] & 0x3F);
    alarm->day = bcd_to_dec(data[2] & 0x3F);
    alarm->weekday = bcd_to_dec(data[3] & 0x07);

    if ((data[0] & 0x80) && (data[1] & 0x80)) {
        *mode = BM8563_ALARM_DISABLED;
        alarm->active = false;
    } else if (!(data[2] & 0x80)) {
        *mode = BM8563_ALARM_ONETIME;
        alarm->active = true;
    } else if (!(data[3] & 0x80)) {
        *mode = BM8563_ALARM_WEEKLY;
        alarm->active = true;
    } else {
        *mode = BM8563_ALARM_DAILY;
        alarm->active = true;
    }

    return ESP_OK;
}

esp_err_t bm8563_clear_alarm_flag(bm8563_handle_t dev) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device");

    uint8_t ctrl2;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, 
                                  (uint8_t[]){BM8563_REG_CTRL2}, 1, 
                                  &ctrl2, 1, -1),
        TAG, "Failed to read CTRL2");

    ctrl2 &= ~0x02;
    uint8_t data[2] = {BM8563_REG_CTRL2, ctrl2};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, data, sizeof(data), -1),
        TAG, "Failed to clear alarm flag");

    return ESP_OK;
}

esp_err_t bm8563_set_timer(bm8563_handle_t dev, uint8_t value, bool timer_interrupt) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device");

    uint8_t timer_ctrl = 0x03; // 1/60Hz
    if (timer_interrupt) {
        timer_ctrl |= 0x80;
    }

    uint8_t timer_data[2] = {BM8563_REG_TIMER_CTRL, timer_ctrl};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, timer_data, sizeof(timer_data), -1),
        TAG, "Failed to set timer control");

    uint8_t value_data[2] = {BM8563_REG_TIMER, value};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, value_data, sizeof(value_data), -1),
        TAG, "Failed to set timer value");

    // Update control register
    uint8_t ctrl2;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, 
                                  (uint8_t[]){BM8563_REG_CTRL2}, 1, 
                                  &ctrl2, 1, -1),
        TAG, "Failed to read CTRL2");

    if (timer_interrupt) {
        ctrl2 |= 0x01;
        dev->timer_enabled = true;
    } else {
        ctrl2 &= ~0x01;
        dev->timer_enabled = false;
    }

    uint8_t ctrl2_data[2] = {BM8563_REG_CTRL2, ctrl2};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, ctrl2_data, sizeof(ctrl2_data), -1),
        TAG, "Failed to update CTRL2");

    return ESP_OK;
}

esp_err_t bm8563_get_timer(bm8563_handle_t dev, uint8_t *value) {
    ESP_RETURN_ON_FALSE(dev && value, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t reg = BM8563_REG_TIMER;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, &reg, 1, value, 1, -1),
        TAG, "Failed to read timer");

    return ESP_OK;
}

esp_err_t bm8563_clear_timer_flag(bm8563_handle_t dev) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device");

    uint8_t ctrl2;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, 
                                  (uint8_t[]){BM8563_REG_CTRL2}, 1, 
                                  &ctrl2, 1, -1),
        TAG, "Failed to read CTRL2");

    ctrl2 &= ~0x01;
    uint8_t data[2] = {BM8563_REG_CTRL2, ctrl2};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, data, sizeof(data), -1),
        TAG, "Failed to clear timer flag");

    return ESP_OK;
}

esp_err_t bm8563_set_clock_out(bm8563_handle_t dev, bm8563_clk_out_freq_t freq) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device");

    uint8_t clkout = 0x00;
    switch (freq) {
        case BM8563_CLK_OUT_32768HZ: clkout = 0x80; break;
        case BM8563_CLK_OUT_1024HZ:  clkout = 0x81; break;
        case BM8563_CLK_OUT_32HZ:    clkout = 0x82; break;
        case BM8563_CLK_OUT_1HZ:     clkout = 0x83; break;
        case BM8563_CLK_OUT_DISABLED: clkout = 0x00; break;
        default: return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2] = {BM8563_REG_CLKOUT, clkout};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, data, sizeof(data), -1),
        TAG, "Failed to set clock out");

    return ESP_OK;
}

esp_err_t bm8563_enable_low_power(bm8563_handle_t dev, bool enable) {
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device");

    uint8_t ctrl1;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit_receive(dev->i2c_dev, 
                                  (uint8_t[]){BM8563_REG_CTRL1}, 1, 
                                  &ctrl1, 1, -1),
        TAG, "Failed to read CTRL1");

    if (enable) {
        ctrl1 |= 0x20;
    } else {
        ctrl1 &= ~0x20;
    }

    uint8_t data[2] = {BM8563_REG_CTRL1, ctrl1};
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(dev->i2c_dev, data, sizeof(data), -1),
        TAG, "Failed to set low power mode");

    return ESP_OK;
}

bool bm8563_check_alarm_flag(bm8563_handle_t dev) {
    if (!dev || dev->int_pin < 0) {
        uint8_t ctrl2;
        if (i2c_master_transmit_receive(dev->i2c_dev, 
                                      (uint8_t[]){BM8563_REG_CTRL2}, 1, 
                                      &ctrl2, 1, -1) == ESP_OK) {
            return (ctrl2 & 0x02);
        }
        return false;
    }
    return gpio_get_level(dev->int_pin) == 0;
}

bool bm8563_check_timer_flag(bm8563_handle_t dev) {
    if (!dev) return false;
    
    uint8_t ctrl2;
    if (i2c_master_transmit_receive(dev->i2c_dev, 
                                  (uint8_t[]){BM8563_REG_CTRL2}, 1, 
                                  &ctrl2, 1, -1) == ESP_OK) {
        return (ctrl2 & 0x01);
    }
    return false;
}