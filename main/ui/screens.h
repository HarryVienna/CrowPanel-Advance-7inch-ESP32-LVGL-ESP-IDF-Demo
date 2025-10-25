#ifndef EEZ_LVGL_UI_SCREENS_H
#define EEZ_LVGL_UI_SCREENS_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _objects_t {
    lv_obj_t *screen_test;
    lv_obj_t *obj0;
    lv_obj_t *input_hours;
    lv_obj_t *input_minutes;
    lv_obj_t *input_seconds;
    lv_obj_t *button_set_time;
    lv_obj_t *obj1;
    lv_obj_t *label_time;
    lv_obj_t *obj2;
    lv_obj_t *slider_brightness;
    lv_obj_t *label_brightness;
    lv_obj_t *number_keyboard;
} objects_t;

extern objects_t objects;

enum ScreensEnum {
    SCREEN_ID_SCREEN_TEST = 1,
};

void create_screen_screen_test();
void tick_screen_screen_test();

void tick_screen_by_id(enum ScreensEnum screenId);
void tick_screen(int screen_index);

void create_screens();


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_SCREENS_H*/