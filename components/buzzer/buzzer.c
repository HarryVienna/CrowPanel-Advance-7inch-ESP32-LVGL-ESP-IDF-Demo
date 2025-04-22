#include "buzzer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Internal configuration and state variables
static const char *TAG = "BUZZER";

// Constant for PWM resolution (10 bit is a good compromise)
#define BUZZER_LEDC_RESOLUTION LEDC_TIMER_10_BIT
// Duty cycle for the beep tone (50% for a square wave)
#define BUZZER_LEDC_DUTY       ((1 << BUZZER_LEDC_RESOLUTION) / 2)
// LEDC Speed Mode (LOW_SPEED_MODE is often sufficient)
#define BUZZER_LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

static gpio_num_t     g_buzzer_pin;
static ledc_timer_t   g_ledc_timer;
static ledc_channel_t g_ledc_channel;
static bool           g_is_initialized = false;

esp_err_t buzzer_init(gpio_num_t buzzer_pin, ledc_timer_t timer_num, ledc_channel_t channel_num) {
    if (g_is_initialized) {
        ESP_LOGW(TAG, "Buzzer already initialized.");
        return ESP_OK; // Or ESP_ERR_INVALID_STATE, depending on the desired behavior
    }

    g_buzzer_pin   = buzzer_pin;
    g_ledc_timer   = timer_num;
    g_ledc_channel = channel_num;

    esp_err_t ret;

    // 1. LEDC Timer Configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = BUZZER_LEDC_SPEED_MODE,
        .duty_resolution = BUZZER_LEDC_RESOLUTION,
        .timer_num       = g_ledc_timer,
        .freq_hz         = 1000, // Initial frequency, will be overwritten by beep()
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // 2. LEDC Channel Configuration
    ledc_channel_config_t ledc_channel = {
        .gpio_num   = g_buzzer_pin,
        .speed_mode = BUZZER_LEDC_SPEED_MODE,
        .channel    = g_ledc_channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = g_ledc_timer,
        .duty       = 0, // Initially off
        .hpoint     = 0,
        .flags.output_invert = 0 // Usually no inversion needed for a buzzer
    };
    ret = ledc_channel_config(&ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel configuration failed: %s", esp_err_to_name(ret));
        // Optional: Revert timer? Probably not necessary.
        return ret;
    }

    g_is_initialized = true;
    ESP_LOGI(TAG, "Buzzer initialized on GPIO %d, Timer %d, Channel %d", g_buzzer_pin, g_ledc_timer, g_ledc_channel);
    return ESP_OK;
}

esp_err_t buzzer_beep(uint32_t frequency, uint32_t duration_ms) {
    if (!g_is_initialized) {
        ESP_LOGE(TAG, "Buzzer not initialized. Call buzzer_init() first.");
        return ESP_ERR_INVALID_STATE;
    }
     if (frequency == 0 || duration_ms == 0) {
        ESP_LOGW(TAG, "Frequency or duration is 0, no beep generated.");
        return buzzer_stop(); // Ensure the buzzer is off
    }

    esp_err_t ret;

    // Set frequency
    ret = ledc_set_freq(BUZZER_LEDC_SPEED_MODE, g_ledc_timer, frequency);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error setting frequency: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set duty cycle (50% for beep tone)
    ret = ledc_set_duty(BUZZER_LEDC_SPEED_MODE, g_ledc_channel, BUZZER_LEDC_DUTY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error setting duty cycle: %s", esp_err_to_name(ret));
        return ret;
    }

    // Apply duty cycle update (starts the tone)
    ret = ledc_update_duty(BUZZER_LEDC_SPEED_MODE, g_ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error updating duty cycle: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for the specified duration
    // pdMS_TO_TICKS converts milliseconds to FreeRTOS ticks
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Stop beep tone
    return buzzer_stop();
}

esp_err_t buzzer_stop(void) {
     if (!g_is_initialized) {
         // Not necessarily an error if stop() is called before init().
         // ESP_LOGW(TAG, "Buzzer not initialized, stop command ignored.");
         return ESP_OK;
     }
     esp_err_t ret;
     // Set duty cycle to 0
    ret = ledc_set_duty(BUZZER_LEDC_SPEED_MODE, g_ledc_channel, 0);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Error setting duty cycle to 0: %s", esp_err_to_name(ret));
         return ret;
     }
    // Apply duty cycle update (stops the tone)
    ret = ledc_update_duty(BUZZER_LEDC_SPEED_MODE, g_ledc_channel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error updating duty cycle (Stop): %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}