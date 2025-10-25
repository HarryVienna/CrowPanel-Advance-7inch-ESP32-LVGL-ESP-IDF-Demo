#ifndef EEZ_LVGL_UI_EVENTS_H
#define EEZ_LVGL_UI_EVENTS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void action_slider_set_backlight_brightness(lv_event_t * e);
extern void action_button_set_time(lv_event_t * e);
extern void action_input_hours(lv_event_t * e);
extern void action_input_minutes(lv_event_t * e);
extern void action_input_seconds(lv_event_t * e);


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_EVENTS_H*/