#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "lvgl.h"
#include "esp32_s3.h"
#include "pca9557.h"


#include "elecrow_advanced_7inch_800x480.h"


#define CONFIG_DOUBLE_FB 0 // Set to 1 for double framebuffer, 0 for single framebuffer

#if CONFIG_DOUBLE_FB
#define LCD_NUM_FB             2
#else
#define LCD_NUM_FB             1
#endif


static const char* TAG = "DISPLAY";

static esp_lcd_touch_handle_t touch_handle = NULL;  
static esp_lcd_panel_handle_t lcd_handle = NULL;    
static pca9557_handle_t expander_handle = NULL;   
static bm8563_handle_t rtc_handle = NULL;   


static void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
static bool on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data);
static void lvgl_port_task(void *arg);

SemaphoreHandle_t lvgl_mux;
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;



/**
 * @brief Initialize Display
 *
 * This function initializes the LCD display, including the backlight, touch panel, LCD panel, and LVGL library.
 */
void init_display(void)
{

    i2c_master_bus_handle_t i2c_handle = NULL;

    init_buzzer();
    init_i2c(&i2c_handle);
    init_i2c_expander(i2c_handle, &expander_handle);
    init_touch(i2c_handle, expander_handle, &touch_handle);
    init_rtc(i2c_handle, &rtc_handle);
    init_lcd(&lcd_handle);
    init_lvgl(lcd_handle, touch_handle);
    init_backlight(expander_handle);

}

/**
 * @brief Initialize LCD Backlight
 *
 * This function initializes the PWM timer and channel configurations for controlling the LCD backlight.
 */
void init_buzzer(void) {

    esp_err_t ret = buzzer_init(BUZZER_GPIO, BUZZER_LEDC_TIMER, BUZZER_LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Buzzer konnte nicht initialisiert werden!");
    }
}

void init_i2c(i2c_master_bus_handle_t *i2c_bus_handle) {
    // 1. I2C Master Config erstellen
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        .flags.enable_internal_pullup = true,
        .flags.allow_pd = false,
        .trans_queue_depth = 0,
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Bus konnte nicht erstellt werden: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_master_bus_reset(*i2c_bus_handle); // I2C Bus zurücksetzen
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C Bus konnte nicht zurückgesetzt werden: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "I2C Bus erfolgreich initialisiert!");
}


/**
 * @brief Initialize LCD Backlight
 *
 * This function initializes the PWM timer and channel configurations for controlling the LCD backlight.
 */
void init_i2c_expander(i2c_master_bus_handle_t i2c_bus_handle, pca9557_handle_t *expander_handle) { 
    // Initialisierung des PCA9557 Treibers
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCA9557_I2C_ADDRESS,
        .scl_speed_hz = I2C_CLK_SPEED_HZ, // Standard I2C Geschwindigkeit
    };

    esp_err_t ret = pca9557_init(i2c_bus_handle, expander_handle, &dev_config); // PCA9557 Treiber initialisieren [5]
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Initialisieren des PCA9557");
        //i2c_master_bus_delete(i2c_bus); // Bei Fehler I2C Bus wieder freigeben
        return;
    }

 
    ESP_LOGI(TAG, "PCA9557 erfolgreich initialisiert");
}

void init_rtc(i2c_master_bus_handle_t i2c_bus_handle, bm8563_handle_t *rtc_handle) { 
    // Initialisierung des PCA9557 Treibers
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BM8563_I2C_ADDR,
        .scl_speed_hz = I2C_CLK_SPEED_HZ, 
        .flags.disable_ack_check = true, // Disable ACK check for the RTC
    };

    esp_err_t ret = bm8563_init(i2c_bus_handle, rtc_handle, &dev_config, GPIO_NUM_NC); // PCA9557 Treiber initialisieren [5]
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Initialisieren des PCA9557");
        //i2c_master_bus_delete(i2c_bus); // Bei Fehler I2C Bus wieder freigeben
        return;
    }

 
    ESP_LOGI(TAG, "bm8563 erfolgreich initialisiert");
}


/**
 * @brief Initialize Touch Driver
 *
 * This function installs the touch driver, configures the I2C interface for touch communication,
 * initializes the touch controller, and creates a touch handle for touch input.
 *
 * @param[out] touch_handle Pointer to the handle for the initialized touch controller.
 */
void init_touch(i2c_master_bus_handle_t i2c_bus_handle, pca9557_handle_t expander_handle, esp_lcd_touch_handle_t *touch_handle) {

    ESP_LOGI(TAG, "Install Touch driver");
    
    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, 
        .int_gpio_num = GPIO_NUM_1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    //const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    esp_lcd_panel_io_i2c_config_t tp_io_config = {                                       
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS, 
        .scl_speed_hz = I2C_CLK_SPEED_HZ,
        .control_phase_bytes = 1,           
        .dc_bit_offset = 0,                 
        .lcd_cmd_bits = 16,                 
        .flags =                            
        {                                   
            .disable_control_phase = 1,     
        }                                       
      };

      
    ESP_LOGI(TAG, "Create LCD panel IO handle");
    esp_lcd_new_panel_io_i2c_v2(i2c_bus_handle, &tp_io_config, &tp_io_handle);
    ESP_LOGI(TAG, "Create a new GT911 touch driver");
    esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, touch_handle, expander_handle);
}


/**
 * @brief Initialize RGB LCD Panel
 *
 * This function installs the RGB LCD panel driver, creates semaphores for synchronization,
 * configures the RGB LCD panel with the provided parameters, registers event callbacks,
 * resets and initializes the RGB LCD panel.
 *
 * @param[out] panel_handle Pointer to the handle for the initialized RGB LCD panel.
 */
void init_lcd(esp_lcd_panel_handle_t *panel_handle) {
    
    ESP_LOGI(TAG, "Install RGB LCD panel driver");

    sem_vsync_end = xSemaphoreCreateBinary();
    sem_gui_ready = xSemaphoreCreateBinary();

    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = LCD_NUM_FB,
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = PIN_NUM_DISP_EN,
        .pclk_gpio_num = PIN_NUM_PCLK,
        .vsync_gpio_num = PIN_NUM_VSYNC,
        .hsync_gpio_num = PIN_NUM_HSYNC,
        .de_gpio_num = PIN_NUM_DE,
        .data_gpio_nums = {
            PIN_NUM_DATA0,
            PIN_NUM_DATA1,
            PIN_NUM_DATA2,
            PIN_NUM_DATA3,
            PIN_NUM_DATA4,
            PIN_NUM_DATA5,
            PIN_NUM_DATA6,
            PIN_NUM_DATA7,
            PIN_NUM_DATA8,
            PIN_NUM_DATA9,
            PIN_NUM_DATA10,
            PIN_NUM_DATA11,
            PIN_NUM_DATA12,
            PIN_NUM_DATA13,
            PIN_NUM_DATA14,
            PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_back_porch = HSYNC_BACK_PORCH,
            .hsync_front_porch = HSYNC_FRONT_PORCH,
            .hsync_pulse_width = HSYNC_PULSE_WIDTH,
            .vsync_back_porch = VSYNC_BACK_PORCH,
            .vsync_front_porch = VSYNC_FRONT_PORCH,
            .vsync_pulse_width = VSYNC_PULSE_WIDTH,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_LOGI(TAG, "Create RGB LCD panel");
    esp_lcd_new_rgb_panel(&panel_config, panel_handle);

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = on_vsync_event,
    };
    esp_lcd_rgb_panel_register_event_callbacks(*panel_handle, &cbs, NULL);


    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    esp_lcd_panel_reset(*panel_handle);
    esp_lcd_panel_init(*panel_handle);
}


void init_backlight(pca9557_handle_t expander_handle) {
    ESP_LOGI(TAG, "Switch on the backlight");

    pca9557_set_direction(expander_handle, 1, PCA9557_OUTPUT); 
    pca9557_write_pin(expander_handle, 1, 1);

    //xTaskCreate(brightness_task, "BRIGHTNESS", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
}



/**
 * @brief Initialize LVGL Library
 *
 * This function initializes the LVGL library, allocates separate draw buffers from PSRAM,
 * registers the display driver and input device driver to LVGL, creates a semaphore for
 * LVGL synchronization, and starts the LVGL port task.
 *
 * @param[in] panel_handle Handle to the LCD panel associated with LVGL.
 * @param[in] touch_handle Handle to the touchpad device associated with LVGL.
 */
void init_lvgl(esp_lcd_panel_handle_t panel_handle, esp_lcd_touch_handle_t touch_handle) {

    ESP_LOGI(TAG, "Initialize LVGL library");

    lvgl_mux = xSemaphoreCreateMutex();

    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
    
    lv_init();

    void *buf1 = NULL;
    void *buf2 = NULL;

    #if CONFIG_DOUBLE_FB
        ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
        ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
        // initialize LVGL draw buffers
        lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * LCD_V_RES);
    #else
        ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
        buf1 = heap_caps_malloc(LCD_H_RES * 10 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
        // initialize LVGL draw buffers
        lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 10);
    #endif 

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    #if CONFIG_DOUBLE_FB
        disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
    #endif
    lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Register input device driver to LVGL");
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_drv.user_data = touch_handle;
    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Start lv_timer_handler task");

    xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 1);
}

/**
 * @brief Touchpad Read Function
 *
 * This function reads touchpad input and updates the LVGL input device data accordingly.
 *
 * @param[in] indev_driver Pointer to the LVGL input device driver structure.
 * @param[out] data Pointer to the LVGL input device data structure to be updated.
 */
static void touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{

    esp_lcd_touch_handle_t touch_handle = (esp_lcd_touch_handle_t)indev_driver->user_data;

    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint16_t touch_strength;
    uint8_t touch_cnt = 0;

    data->state = LV_INDEV_STATE_REL;

    esp_lcd_touch_read_data(touch_handle);
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_handle, &touchpad_x, &touchpad_y, &touch_strength, &touch_cnt, 1);
    if (touchpad_pressed) {
        //ESP_LOGI(TAG, "Touchpad_read %d %d", touchpad_x, touchpad_y);
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
    }
}

/**
 * @brief LVGL Flush Callback
 *
 * This callback function is called by LVGL to flush a portion of the display buffer to the physical display.
 * It passes the draw buffer to the LCD panel driver, indicating the area that needs to be updated.
 *
 * @param[in] drv Pointer to the display driver structure.
 * @param[in] area Pointer to the area that needs to be flushed.
 * @param[in] color_map Pointer to the color map containing pixel data to be flushed.
 */
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // LVGL has finished
    xSemaphoreGive(sem_gui_ready);
    // Now wait for the VSYNC event. 
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);

    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

/**
 * @brief Handles VSYNC events for an ESP32 LCD panel.
 *
 * This function waits until LVGL has finished its operations, indicated by the
 * `sem_gui_ready` semaphore being taken. Once LVGL is ready, it signals that
 * the VSYNC event has ended by giving the `sem_vsync_end` semaphore.
 *
 * @param[in] panel Handle to the LCD panel associated with the VSYNC event.
 * @param[in] event_data Pointer to a structure containing data related to the VSYNC event.
 * @param[in] user_data User data pointer passed when registering the VSYNC event handler.
 * @return
 *     - `true` if the task was woken up due to handling the VSYNC event, indicating that it's safe
 *       to proceed with flushing the buffer.
 *     - `false` otherwise.
 */
static bool on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    // Wait until LVGL has finished 
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        // Indicate that the VSYNC event has ended, and it's safe to proceed with flushing the buffer.
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }

    return high_task_awoken == pdTRUE;
}

/**
 * @brief LVGL Port Task
 *
 * This task handles LVGL operations in the background. It periodically calls
 * the LVGL timer handler to update the GUI.
 *
 * @param[in] arg Pointer to task arguments (not used).
 */
static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");

    while (1) {

        xSemaphoreTake(lvgl_mux, portMAX_DELAY);
        lv_timer_handler();
        xSemaphoreGive(lvgl_mux);

        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_DELAY_MS));
    }
}

void set_time(uint8_t hours, uint8_t minutes, uint8_t seconds) {

   
    struct tm time_to_set = {
        .tm_sec = seconds,
        .tm_min = minutes,
        .tm_hour = hours,
        .tm_mday = 15,
        .tm_mon = 3,    // April (0-basiert)
        .tm_year = 124, // 2024 (Jahre seit 1900)
        .tm_wday = 1    // Montag
    };

    ESP_LOGI(TAG, "Setze Uhrzeit auf: %02d.%02d.%04d %02d:%02d:%02d",
             time_to_set.tm_mday,
             time_to_set.tm_mon + 1,
             time_to_set.tm_year + 1900,
             time_to_set.tm_hour,
             time_to_set.tm_min,
             time_to_set.tm_sec);

    ESP_ERROR_CHECK(bm8563_set_time(rtc_handle, &time_to_set));
    ESP_LOGI(TAG, "Uhrzeit erfolgreich gesetzt!");
}

esp_err_t get_time(uint8_t *hours, uint8_t *minutes, uint8_t *seconds) {
    struct tm current_time;

    esp_err_t ret = bm8563_get_time(rtc_handle, &current_time);
    if (ret != ESP_OK) {
        return ret;
    }

    *hours = current_time.tm_hour;
    *minutes = current_time.tm_min;
    *seconds = current_time.tm_sec;

    return ESP_OK;
}