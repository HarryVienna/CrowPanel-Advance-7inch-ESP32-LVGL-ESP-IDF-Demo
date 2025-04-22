#ifndef BUZZER_H_
#define BUZZER_H_

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the buzzer driver with the specified GPIO pin, LEDC timer, and channel.
 *
 * @param buzzer_pin The GPIO pin the buzzer is connected to.
 * @param timer_num The LEDC timer to use (e.g., LEDC_TIMER_0).
 * @param channel_num The LEDC channel to use (e.g., LEDC_CHANNEL_0).
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t buzzer_init(gpio_num_t buzzer_pin, ledc_timer_t timer_num, ledc_channel_t channel_num);

/**
 * @brief Generates a beep tone with the specified frequency and duration.
 *
 * IMPORTANT: This function blocks for the duration of the beep!
 *
 * @param frequency The frequency of the beep in Hz.
 * @param duration_ms The duration of the beep in milliseconds.
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t buzzer_beep(uint32_t frequency, uint32_t duration_ms);

/**
 * @brief Turns the buzzer off immediately (sets duty cycle to 0).
 *
 * @return ESP_OK on success, otherwise an error code.
 */
esp_err_t buzzer_stop(void);


#ifdef __cplusplus
}
#endif

#endif // BUZZER_H_