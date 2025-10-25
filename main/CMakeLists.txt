# main/CMakeLists.txt

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources} 
                       INCLUDE_DIRS "."
                       REQUIRES lvgl stc8h1k28 bm8563 esp_lcd_touch esp_lcd_touch_gt911)