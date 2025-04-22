#ifndef GUI_H
#define GUI_H

#include "lvgl.h"
#include "../ui/ui.h"
#include "../display/esp32_s3.h"



void disp_time();

void event_screen_init(lv_event_t *e);

void event_slider_set_hours(lv_event_t * e);
void event_slider_set_minutes(lv_event_t * e);
void event_slider_set_seconds(lv_event_t * e);


#endif /* GUI_H */