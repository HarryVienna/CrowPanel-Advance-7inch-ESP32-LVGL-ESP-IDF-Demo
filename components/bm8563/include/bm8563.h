#pragma once
#include <time.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BM8563_I2C_ADDR 0x51

typedef enum {
    BM8563_CLK_OUT_32768HZ = 0,
    BM8563_CLK_OUT_1024HZ,
    BM8563_CLK_OUT_32HZ,
    BM8563_CLK_OUT_1HZ,
    BM8563_CLK_OUT_DISABLED
} bm8563_clk_out_freq_t;

typedef enum {
    BM8563_ALARM_DISABLED = 0,
    BM8563_ALARM_DAILY,
    BM8563_ALARM_WEEKLY,
    BM8563_ALARM_ONETIME
} bm8563_alarm_mode_t;

typedef struct {
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t weekday;
    bool active;
} bm8563_alarm_t;

typedef struct bm8563_dev_t {
    i2c_master_dev_handle_t i2c_dev;
    gpio_num_t int_pin;
    bool alarm_enabled;
    bool timer_enabled;
} bm8563_dev_t;

// Typdefinition für das Geräte-Handle
typedef struct bm8563_dev_t *bm8563_handle_t;


/**
 * @brief Initialize the BM8563 RTC device
 *
 * This function:
 * 1. Allocates memory for the device handle
 * 2. Configures the I2C communication
 * 3. Sets up the interrupt pin (if specified)
 * 4. Prepares the RTC with default configuration (clock running, alarms disabled)
 *
 * @param bus_handle Initialized I2C bus handle (from i2c_master_bus_init())
 * @param[out] rtc_handle Pointer to store the created device handle
 * @param config I2C device configuration (address, scl_speed etc.)
 * @param int_pin GPIO number for interrupt pin (use GPIO_NUM_NC if not used)
 * 
 * @return
 *        - ESP_OK: Initialization successful
 *        - ESP_ERR_INVALID_ARG: Invalid parameters
 *        - ESP_ERR_NO_MEM: Memory allocation failed
 *        - ESP_ERR_NOT_FOUND: Device not found on I2C bus
 *        - ESP_FAIL: Other initialization error
 *
 * @note
 * - The device handle MUST be freed with bm8563_deinit() when no longer needed
 * - For battery-backed operation, ensure VBAT is connected
 * - Default initialization:
 *   - Clock starts running
 *   - Alarms/timer disabled
 *   - Clock output disabled
 *   - 24-hour mode
 *
 * @code
 * // Example initialization:
 * i2c_device_config_t dev_cfg = {
 *     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
 *     .device_address = BM8563_I2C_ADDR,
 *     .scl_speed_hz = 100000,
 * };
 * bm8563_handle_t rtc;
 * ESP_ERROR_CHECK(bm8563_init(i2c_bus, &rtc, &dev_cfg, GPIO_NUM_15));
 * @endcode
 */
esp_err_t bm8563_init(i2c_master_bus_handle_t bus_handle, bm8563_handle_t *rtc_handle, const i2c_device_config_t *config, gpio_num_t int_pin);

/**
 * @brief Deinitialize the BM8563 RTC device and free resources
 * 
 * This function:
 * 1. Removes the device from the I2C bus
 * 2. Releases all allocated memory
 * 3. Invalidates the device handle
 *
 * @param dev Device handle to deinitialize (pointer becomes invalid after this call)
 *
 * @note
 * - Safe to call with NULL handle (no operation performed)
 * - Must be called for every initialized device to prevent memory leaks
 * - Does NOT modify RTC registers (time/configuration persists)
 * - Interrupt GPIO configuration remains unchanged
 *
 * @warning
 * - The device handle becomes invalid after this call
 * - Any further use of the handle will cause undefined behavior
 *
 * @code
 * // Cleanup example:
 * bm8563_handle_t rtc;
 * // ... initialization and usage ...
 * bm8563_deinit(rtc);
 * // rtc is now INVALID and must not be used
 * @endcode
 */
void bm8563_deinit(bm8563_handle_t dev);

/**
 * @brief Set the current time and date on the BM8563 RTC
 *
 * Configures the RTC with the provided time structure. This includes:
 * - Seconds (0-59)
 * - Minutes (0-59)
 * - Hours (0-23, 24-hour format)
 * - Day of month (1-31)
 * - Month (0-11, 0 = January)
 * - Year (years since 1900, e.g. 124 for 2024)
 * - Weekday (0-6, 0 = Sunday)
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param time Pointer to tm structure containing time/date to set
 * 
 * @return 
 *        - ESP_OK: Time successfully set
 *        - ESP_ERR_INVALID_ARG: Invalid parameters or time values out of range
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - Automatically handles leap years until year 2099
 * - Sets the RTC's internal clock running (STOP bit cleared)
 * - Time is converted to BCD format before writing
 * - Weekday value is calculated if not provided (but recommended to set correctly)
 *
 * @warning
 * - Year value must be >= 0 (years since 1900)
 * - Month range is 0-11 (unlike struct tm in some implementations)
 *
 * @code
 * // Example: Set to 15th April 2024, 14:30:00 (Monday)
 * struct tm new_time = {
 *     .tm_sec = 0,
 *     .tm_min = 30,
 *     .tm_hour = 14,
 *     .tm_mday = 15,
 *     .tm_mon = 3,    // April (0-based)
 *     .tm_year = 124, // 2024 (1900 + 124)
 *     .tm_wday = 1    // Monday
 * };
 * ESP_ERROR_CHECK(bm8563_set_time(rtc, &new_time));
 * @endcode
 */
esp_err_t bm8563_set_time(bm8563_handle_t dev, const struct tm *time);

/**
 * @brief Read the current time and date from the BM8563 RTC
 * 
 * Retrieves the current time from the RTC and populates a tm structure with:
 * - Seconds (0-59)
 * - Minutes (0-59)
 * - Hours (0-23, 24-hour format)
 * - Day of month (1-31)
 * - Month (0-11, 0 = January)
 * - Year (years since 1900)
 * - Weekday (0-6, 0 = Sunday)
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param[out] time Pointer to tm structure to store the retrieved time
 * 
 * @return 
 *        - ESP_OK: Time successfully read
 *        - ESP_ERR_INVALID_ARG: Invalid parameters (NULL pointer)
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - The RTC maintains time in BCD format - automatic conversion to binary is performed
 * - Leap year compensation is automatically handled by the RTC hardware
 * - For proper weekday calculation, ensure VBAT backup is connected
 * - The tm structure will be fully populated (all fields set, even if unused)
 *
 * @warning
 * - The tm_year field returns years since 1900 (e.g., 124 for 2024)
 * - The tm_mon field is 0-based (0 = January, 11 = December)
 * - Time is only accurate if the RTC oscillator is running (check STOP bit if time appears frozen)
 *
 * @code
 * // Example usage:
 * struct tm current_time;
 * if (bm8563_get_time(rtc, &current_time) == ESP_OK) {
 *     ESP_LOGI(TAG, "Current time: %02d:%02d:%02d", 
 *             current_time.tm_hour, 
 *             current_time.tm_min, 
 *             current_time.tm_sec);
 * }
 * @endcode
 */
esp_err_t bm8563_get_time(bm8563_handle_t dev, struct tm *time);

/**
 * @brief Configure an alarm on the BM8563 RTC
 * 
 * Sets up an alarm with configurable mode and timing. The BM8563 supports:
 * - Daily alarms (hour/minute match)
 * - Weekly alarms (weekday/hour/minute match)
 * - One-time alarms (day/hour/minute match)
 * - Complete alarm disabling
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param alarm Pointer to alarm configuration structure containing:
 *              - minutes: 0-59
 *              - hours: 0-23 (24-hour format)
 *              - day: 1-31 (for BM8563_ALARM_ONETIME)
 *              - weekday: 0-6 (0=Sunday, for BM8563_ALARM_WEEKLY)
 *              - active: Enable/disable the alarm
 * @param mode Alarm mode (see bm8563_alarm_mode_t):
 *             - BM8563_ALARM_DAILY
 *             - BM8563_ALARM_WEEKLY  
 *             - BM8563_ALARM_ONETIME
 *             - BM8563_ALARM_DISABLED
 *
 * @return
 *        - ESP_OK: Alarm successfully configured
 *        - ESP_ERR_INVALID_ARG: Invalid parameters or time values out of range
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - For weekly alarms: weekday must be 0-6 (0=Sunday)
 * - For one-time alarms: day must be 1-31
 * - The INT pin will trigger when:
 *   - Alarm matches AND interrupt is enabled
 *   - Flag must be cleared with bm8563_clear_alarm_flag()
 * - Alarm remains active even after trigger until disabled
 *
 * @warning 
 * - Does not automatically enable interrupts (use bm8563_clear_alarm_flag() after configuration)
 * - Weekly/one-time modes require correct weekday/day values to trigger
 *
 * @code
 * // Example: Daily alarm at 07:30
 * bm8563_alarm_t alarm = {
 *     .minutes = 30,
 *     .hours = 7,
 *     .active = true
 * };
 * ESP_ERROR_CHECK(bm8563_set_alarm(rtc, &alarm, BM8563_ALARM_DAILY));
 * @endcode
 */

esp_err_t bm8563_set_alarm(bm8563_handle_t dev, const bm8563_alarm_t *alarm, bm8563_alarm_mode_t mode);
/**
 * @brief Read the current alarm configuration from the BM8563
 *
 * Retrieves the configured alarm settings and operating mode. The function:
 * - Decodes BCD values from alarm registers to binary
 * - Determines the current alarm mode
 * - Checks the actual interrupt enable status
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param[out] alarm Pointer to structure to store alarm settings
 * @param[out] mode Pointer to store current alarm mode
 *
 * @return
 *        - ESP_OK: Alarm settings successfully read
 *        - ESP_ERR_INVALID_ARG: Invalid parameters (NULL pointers)
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - The 'active' field reflects the hardware interrupt enable state
 * - Register values are automatically converted from BCD
 * - Returns actual configured values, not remaining time
 *
 * @warning
 * - Day/weekday values may be invalid if alarm is in daily mode
 * - The alarm might have triggered even if 'active' is true (check flag)
 *
 * @code
 * // Example: Read current alarm
 * bm8563_alarm_t alarm;
 * bm8563_alarm_mode_t mode;
 * if (bm8563_get_alarm(rtc, &alarm, &mode) == ESP_OK) {
 *     ESP_LOGI(TAG, "Alarm set for %02d:%02d (Mode: %d)", 
 *             alarm.hours, alarm.minutes, mode);
 * }
 * @endcode
 */
esp_err_t bm8563_get_alarm(bm8563_handle_t dev, bm8563_alarm_t *alarm, bm8563_alarm_mode_t *mode);

/**
 * @brief Clear the alarm interrupt flag on the BM8563 RTC
 *
 * Resets the alarm flag (AF) in the control register after an alarm trigger.
 * This function:
 * - Clears Bit 1 (AF) in Control Register 2 (0x01)
 * - Resets the interrupt state on the INT pin (if connected)
 * - Allows new alarm interrupts to be recognized
 *
 * @param dev Initialized device handle from bm8563_init()
 * 
 * @return
 *        - ESP_OK: Flag successfully cleared
 *        - ESP_ERR_INVALID_ARG: Invalid device handle
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - Must be called after handling an alarm interrupt
 * - Does not disable future alarms (only clears the flag)
 * - Safe to call even when no alarm has triggered
 * - If using the INT pin: Pin state returns high after clearing
 *
 * @warning
 * - Repeated alarms will retrigger until disabled via bm8563_set_alarm()
 * - Failure to clear may prevent subsequent alarms from triggering
 *
 * @code
 * // Typical usage in an ISR:
 * void alarm_isr(void *arg) {
 *     bm8563_handle_t rtc = (bm8563_handle_t)arg;
 *     if (bm8563_check_alarm_flag(rtc)) {
 *         ESP_LOGI(TAG, "Alarm triggered!");
 *         ESP_ERROR_CHECK(bm8563_clear_alarm_flag(rtc));
 *     }
 * }
 * @endcode
 */
esp_err_t bm8563_clear_alarm_flag(bm8563_handle_t dev);

/**
 * @brief Configure the countdown timer on the BM8563 RTC
 * 
 * Sets up a countdown timer with selectable duration and interrupt control.
 * The timer can operate in four frequency modes (1/4096Hz to 1/60Hz).
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param value Timer value (1-255) in selected resolution
 * @param timer_interrupt Enable/disable timer interrupt generation
 * 
 * @return 
 *        - ESP_OK: Timer successfully configured
 *        - ESP_ERR_INVALID_ARG: Invalid device handle
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - Default configuration uses 1/60Hz resolution (1 minute per count)
 * - Timer starts immediately after configuration
 * - For 1/60Hz mode: value = minutes (max 255 minutes/4.25 hours)
 * - Interrupt triggers when timer reaches zero
 * - Flag must be cleared with bm8563_clear_timer_flag()
 *
 * @warning
 * - Timer continues running in low-power mode (STOP bit has no effect)
 * - Value outside 1-255 range will be truncated
 * - Ensure INT pin is configured if using interrupts
 *
 * @code
 * // Example: 5-minute timer with interrupt
 * ESP_ERROR_CHECK(bm8563_set_timer(rtc, 5, true));
 * @endcode
 */
esp_err_t bm8563_set_timer(bm8563_handle_t dev, uint8_t value, bool timer_interrupt);

/**
 * @brief Read the current timer value from the BM8563
 *
 * Retrieves the remaining countdown value from the timer register.
 * The returned value represents counts remaining at the configured frequency.
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param[out] value Pointer to store the timer value (1-255)
 * 
 * @return 
 *        - ESP_OK: Value successfully read
 *        - ESP_ERR_INVALID_ARG: Invalid parameters
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - Value decreases in real-time until reaching zero
 * - Does not indicate if timer has triggered (check flag separately)
 * - Returns raw register value without frequency context
 * - Accuracy depends on 32.768kHz crystal stability
 *
 * @warning
 * - Value resets to initial setting after reaching zero
 * - Reading during countdown may return transient values
 *
 * @code
 * // Example: Monitor remaining time
 * uint8_t remaining;
 * if (bm8563_get_timer(rtc, &remaining) == ESP_OK) {
 *     ESP_LOGI(TAG, "Timer remaining: %d counts", remaining);
 * }
 * @endcode
 */
esp_err_t bm8563_get_timer(bm8563_handle_t dev, uint8_t *value);

/**
 * @brief Clear the timer interrupt flag on the BM8563 RTC
 * 
 * Resets the timer flag (TF) in the control register after a timer trigger.
 * This function:
 * - Clears Bit 0 (TF) in Control Register 2 (0x01)
 * - Resets the interrupt state if using the INT pin
 * - Allows new timer interrupts to be recognized
 *
 * @param dev Initialized device handle from bm8563_init()
 * 
 * @return 
 *        - ESP_OK: Flag successfully cleared
 *        - ESP_ERR_INVALID_ARG: Invalid device handle
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - Must be called after handling a timer interrupt
 * - Does not stop or reset the timer (only clears the flag)
 * - Safe to call even when no timer has triggered
 *
 * @warning
 * - Timer continues running after flag clearance
 * - Repeated interrupts occur if timer is not reconfigured
 *
 * @code
 * // Typical usage in ISR:
 * void timer_isr(void *arg) {
 *     bm8563_handle_t rtc = (bm8563_handle_t)arg;
 *     if (bm8563_check_timer_flag(rtc)) {
 *         ESP_ERROR_CHECK(bm8563_clear_timer_flag(rtc));
 *     }
 * }
 * @endcode
 */
esp_err_t bm8563_clear_timer_flag(bm8563_handle_t dev);

/**
 * @brief Set the clock output frequency of the BM8563 RTC
 *
 * Configures the CLKOUT pin to generate a square wave at one of the available frequencies.
 * The signal can be used as a clock source for external components or as a periodic interrupt.
 *
 * @param dev      Device handle initialized with bm8563_init()
 * @param freq     Frequency to set (see bm8563_clk_out_freq_t for options)
 * 
 * @return 
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid parameters (null handle or unsupported frequency)
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - The CLKOUT pin must be properly connected to utilize this feature
 * - Disabling the output (BM8563_CLK_OUT_DISABLED) reduces power consumption
 * - Frequency accuracy depends on the RTC's 32.768 kHz crystal
 *
 * @code
 * // Example: Enable 1Hz clock output
 * bm8563_set_clock_out(rtc, BM8563_CLK_OUT_1HZ);
 * @endcode
 */
esp_err_t bm8563_set_clock_out(bm8563_handle_t dev, bm8563_clk_out_freq_t freq);

/**
 * @brief Enable/disable low-power mode (STOP bit control)
 *
 * Controls the STOP bit in Control Register 1 (0x00):
 * - When enabled (STOP=1): RTC clock halts (timekeeping stops)
 * - When disabled (STOP=0): Normal operation (clock runs)
 *
 * @param dev Initialized device handle from bm8563_init()
 * @param enable true to enter low-power mode, false for normal operation
 * 
 * @return 
 *        - ESP_OK: Mode successfully set
 *        - ESP_ERR_INVALID_ARG: Invalid device handle
 *        - ESP_FAIL: I2C communication error
 *
 * @note
 * - Timer continues running in low-power mode
 * - All registers retain their values
 * - Timekeeping resumes from stopped time when disabled
 *
 * @warning
 * - Do not use for power-critical applications (VBAT domain remains active)
 * - Real-time clock accuracy may drift after STOP mode
 *
 * @code
 * // Enter low-power mode:
 * ESP_ERROR_CHECK(bm8563_enable_low_power(rtc, true));
 * @endcode
 */
esp_err_t bm8563_enable_low_power(bm8563_handle_t dev, bool enable);

/**
 * @brief Check the alarm interrupt flag status
 *
 * Determines if an alarm has triggered by either:
 * - Reading the INT pin state (if configured)
 * - Checking Bit 1 (AF) in Control Register 2 (0x01)
 *
 * @param dev Initialized device handle from bm8563_init()
 * 
 * @return
 *        - true: Alarm has triggered
 *        - false: No alarm trigger or error occurred
 *
 * @note
 * - Prefer INT pin checking when available (lower power)
 * - Flag state persists until cleared
 * - Safe to call frequently (non-blocking)
 *
 * @warning
 * - Returns false if INT pin not configured and I2C fails
 * - Does not distinguish between multiple alarm types
 *
 * @code
 * // Polling example:
 * if (bm8563_check_alarm_flag(rtc)) {
 *     ESP_LOGI(TAG, "Alarm detected!");
 * }
 * @endcode
 */
bool bm8563_check_alarm_flag(bm8563_handle_t dev);

/**
 * @brief Check the timer interrupt flag status
 *
 * Determines if the timer has expired by checking:
 * - Bit 0 (TF) in Control Register 2 (0x01)
 *
 * @param dev Initialized device handle from bm8563_init()
 * 
 * @return
 *        - true: Timer has expired
 *        - false: Timer not expired or error occurred
 *
 * @note
 * - Flag state persists until cleared
 * - Independent of interrupt enable/disable state
 * - Safe for frequent polling (non-blocking)
 *
 * @warning
 * - Returns false on I2C communication failure
 * - Does not auto-clear the flag (must call bm8563_clear_timer_flag())
 *
 * @code
 * // Timer status check:
 * if (bm8563_check_timer_flag(rtc)) {
 *     ESP_LOGI(TAG, "Timer expired at %lld", esp_timer_get_time());
 * }
 * @endcode
 */
bool bm8563_check_timer_flag(bm8563_handle_t dev);

#ifdef __cplusplus
}
#endif