#include "esp_log.h"

#include "gui.h"
#include <esp_err.h>


static const char* TAG = "GUI";

uint8_t hours = 0;
uint8_t minutes = 0;
uint8_t seconds = 0;



void event_screen_init(lv_event_t * e) {
    ESP_LOGI(TAG, "Init screen");
}

void event_slider_set_hours(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    hours = value;

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(ui_LabelHours, str);
}

void event_slider_set_minutes(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    minutes = value;

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(ui_LabelMinutes, str);
}

void event_slider_set_seconds(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    seconds = value;

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(ui_LabelSeconds, str);
}

void event_set_time(lv_event_t * e) {

    ESP_LOGI(TAG, "Set time to %d %d %d", hours, minutes, seconds);

    set_time(hours, minutes, seconds);

}

void disp_time() {

    u_int8_t curr_hours;
    u_int8_t curr_minutes;
    u_int8_t curr_seconds;

    if (get_time(&curr_hours, &curr_minutes, &curr_seconds) == ESP_OK) {
        char str[12];
        sprintf( str, "%02d:%02d:%02d", curr_hours, curr_minutes, curr_seconds );

        lv_label_set_text(ui_LabelTime, str);
    } 
    
}
