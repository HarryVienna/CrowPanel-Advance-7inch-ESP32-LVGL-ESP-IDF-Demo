// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "../ui.h"

void ui_ScreenTest_screen_init(void)
{
    ui_ScreenTest = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ScreenTest, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_flex_flow(ui_ScreenTest, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ui_ScreenTest, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_ScreenTest, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ScreenTest, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel1 = lv_obj_create(ui_ScreenTest);
    lv_obj_set_width(ui_Panel1, 800);
    lv_obj_set_height(ui_Panel1, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_align(ui_Panel1, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Panel1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Panel1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel1, lv_color_hex(0xC0C0C0), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Panel1, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelHelloWorld = lv_label_create(ui_Panel1);
    lv_obj_set_width(ui_LabelHelloWorld, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelHelloWorld, LV_SIZE_CONTENT);    /// 30
    lv_obj_set_align(ui_LabelHelloWorld, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelHelloWorld, "Hello World");
    lv_obj_set_style_text_font(ui_LabelHelloWorld, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelTime = lv_label_create(ui_Panel1);
    lv_obj_set_height(ui_LabelTime, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_flex_grow(ui_LabelTime, 1);
    lv_obj_set_align(ui_LabelTime, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelTime, "12:34:56");
    lv_obj_set_style_text_align(ui_LabelTime, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelTime, &ui_font_Digital48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_ScreenTest);
    lv_obj_set_width(ui_Panel2, 800);
    lv_obj_set_height(ui_Panel2, LV_SIZE_CONTENT);    /// 100
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Panel2, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Panel2, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_SliderHours = lv_slider_create(ui_Panel2);
    lv_slider_set_range(ui_SliderHours, 0, 23);
    lv_slider_set_value(ui_SliderHours, 0, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_SliderHours) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_SliderHours, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_SliderHours, 500);
    lv_obj_set_height(ui_SliderHours, 15);
    lv_obj_set_align(ui_SliderHours, LV_ALIGN_CENTER);

    ui_LabelHours = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_LabelHours, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelHours, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelHours, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelHours, "0");
    lv_obj_set_style_text_font(ui_LabelHours, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_LabelHours, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_LabelHours, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_LabelHours, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_LabelHours, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel3 = lv_obj_create(ui_ScreenTest);
    lv_obj_set_width(ui_Panel3, 800);
    lv_obj_set_height(ui_Panel3, LV_SIZE_CONTENT);    /// 100
    lv_obj_set_align(ui_Panel3, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Panel3, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Panel3, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_SliderMinutes = lv_slider_create(ui_Panel3);
    lv_slider_set_range(ui_SliderMinutes, 0, 59);
    lv_slider_set_value(ui_SliderMinutes, 0, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_SliderMinutes) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_SliderMinutes, 0,
                                                                                                  LV_ANIM_OFF);
    lv_obj_set_width(ui_SliderMinutes, 500);
    lv_obj_set_height(ui_SliderMinutes, 15);
    lv_obj_set_align(ui_SliderMinutes, LV_ALIGN_CENTER);

    ui_LabelMinutes = lv_label_create(ui_Panel3);
    lv_obj_set_width(ui_LabelMinutes, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelMinutes, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelMinutes, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelMinutes, "0");
    lv_obj_set_style_text_font(ui_LabelMinutes, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_LabelMinutes, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_LabelMinutes, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_LabelMinutes, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_LabelMinutes, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel4 = lv_obj_create(ui_ScreenTest);
    lv_obj_set_width(ui_Panel4, 800);
    lv_obj_set_height(ui_Panel4, LV_SIZE_CONTENT);    /// 100
    lv_obj_set_align(ui_Panel4, LV_ALIGN_CENTER);
    lv_obj_set_flex_flow(ui_Panel4, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Panel4, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Panel4, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_SliderSeconds = lv_slider_create(ui_Panel4);
    lv_slider_set_range(ui_SliderSeconds, 0, 59);
    lv_slider_set_value(ui_SliderSeconds, 0, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_SliderSeconds) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_SliderSeconds, 0,
                                                                                                  LV_ANIM_OFF);
    lv_obj_set_width(ui_SliderSeconds, 500);
    lv_obj_set_height(ui_SliderSeconds, 15);
    lv_obj_set_align(ui_SliderSeconds, LV_ALIGN_CENTER);

    ui_LabelSeconds = lv_label_create(ui_Panel4);
    lv_obj_set_width(ui_LabelSeconds, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelSeconds, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelSeconds, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelSeconds, "0");
    lv_obj_set_style_text_font(ui_LabelSeconds, &lv_font_montserrat_40, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_LabelSeconds, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_LabelSeconds, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_LabelSeconds, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_LabelSeconds, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel5 = lv_obj_create(ui_ScreenTest);
    lv_obj_set_width(ui_Panel5, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_height(ui_Panel5, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_align(ui_Panel5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_pad_left(ui_Panel5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Panel5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Panel5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Panel5, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_SetTime = lv_btn_create(ui_Panel5);
    lv_obj_set_width(ui_SetTime, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_height(ui_SetTime, LV_SIZE_CONTENT);    /// 50
    lv_obj_set_align(ui_SetTime, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_SetTime, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_SetTime, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Label2 = lv_label_create(ui_SetTime);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "Set time");
    lv_obj_set_style_text_font(ui_Label2, &lv_font_montserrat_28, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_SliderHours, ui_event_SliderHours, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SliderMinutes, ui_event_SliderMinutes, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SliderSeconds, ui_event_SliderSeconds, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SetTime, ui_event_SetTime, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ScreenTest, ui_event_ScreenTest, LV_EVENT_ALL, NULL);
    uic_SetTime = ui_SetTime;

}
