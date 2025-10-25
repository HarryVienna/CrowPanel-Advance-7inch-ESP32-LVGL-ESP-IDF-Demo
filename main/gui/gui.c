#include "esp_log.h"

#include "gui.h"
#include <esp_err.h>


static const char* TAG = "GUI";




void set_brightness(int32_t value) {

    set_backlight_brightness(value);

    char str[10];
    sprintf( str, "%lu", value );

    lv_label_set_text(objects.label_brightness, str);
}

void disp_time() {

    u_int8_t curr_hours;
    u_int8_t curr_minutes;
    u_int8_t curr_seconds;

    if (get_time(&curr_hours, &curr_minutes, &curr_seconds) == ESP_OK) {
        char str[12];
        sprintf( str, "%02d:%02d:%02d", curr_hours, curr_minutes, curr_seconds );

        lv_label_set_text(objects.label_time, str);
    } 
    
}

// Actions

void action_slider_set_backlight_brightness(lv_event_t * e) {
    lv_obj_t * slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    set_brightness(value);
}

void action_button_set_time(lv_event_t * e){

    const char *hours_value = lv_textarea_get_text(objects.input_hours);
    uint8_t hours = (uint8_t)atoi(hours_value);

    const char *minutes_value = lv_textarea_get_text(objects.input_minutes);
    uint8_t minutes = (uint8_t)atoi(minutes_value);

    const char *seconds_value = lv_textarea_get_text(objects.input_seconds);
    uint8_t seconds = (uint8_t)atoi(seconds_value);

    ESP_LOGI(TAG, "Set time to %d %d %d", hours, minutes, seconds);

    set_time(hours, minutes, seconds);
}

void action_input_hours(lv_event_t * e){
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_FOCUSED) {
        lv_obj_clear_flag(objects.number_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if(event_code == LV_EVENT_DEFOCUSED) {
        lv_obj_add_flag(objects.number_keyboard, LV_OBJ_FLAG_HIDDEN);
    }

    lv_keyboard_set_textarea(objects.number_keyboard, objects.input_hours);
}



void action_input_minutes(lv_event_t * e){
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_FOCUSED) {
        lv_obj_clear_flag(objects.number_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if(event_code == LV_EVENT_DEFOCUSED) {
        lv_obj_add_flag(objects.number_keyboard, LV_OBJ_FLAG_HIDDEN);
    }

    lv_keyboard_set_textarea(objects.number_keyboard, objects.input_minutes);
}

void action_input_seconds(lv_event_t * e){
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_FOCUSED) {
        lv_obj_clear_flag(objects.number_keyboard, LV_OBJ_FLAG_HIDDEN);
    }
    if(event_code == LV_EVENT_DEFOCUSED) {
        lv_obj_add_flag(objects.number_keyboard, LV_OBJ_FLAG_HIDDEN);
    }

    lv_keyboard_set_textarea(objects.number_keyboard, objects.input_seconds);
}
