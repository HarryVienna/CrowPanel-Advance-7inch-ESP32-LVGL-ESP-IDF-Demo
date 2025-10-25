#ifndef EEZ_LVGL_UI_IMAGES_H
#define EEZ_LVGL_UI_IMAGES_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const lv_img_dsc_t img_0d;
extern const lv_img_dsc_t img_0n;
extern const lv_img_dsc_t img_wifi_off;
extern const lv_img_dsc_t img_wifi_on;
extern const lv_img_dsc_t img_cloud;
extern const lv_img_dsc_t img_uv;
extern const lv_img_dsc_t img_arrow;
extern const lv_img_dsc_t img_sunrise;
extern const lv_img_dsc_t img_sunset;
extern const lv_img_dsc_t img_humidity;
extern const lv_img_dsc_t img_pressure;
extern const lv_img_dsc_t img_battery;
extern const lv_img_dsc_t img_clock;

#ifndef EXT_IMG_DESC_T
#define EXT_IMG_DESC_T
typedef struct _ext_img_desc_t {
    const char *name;
    const lv_img_dsc_t *img_dsc;
} ext_img_desc_t;
#endif

extern const ext_img_desc_t images[13];


#ifdef __cplusplus
}
#endif

#endif /*EEZ_LVGL_UI_IMAGES_H*/