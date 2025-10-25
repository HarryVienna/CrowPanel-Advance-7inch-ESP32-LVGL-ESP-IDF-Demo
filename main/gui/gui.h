#ifndef GUI_H
#define GUI_H

#include "lvgl.h"
#include "../ui/ui.h"
#include "../display/esp32_s3.h"



void set_brightness(int32_t value);
void disp_time();

void event_slider_set_backlight_brightness(lv_event_t * e);



#endif /* GUI_H */