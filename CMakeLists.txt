# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
list(APPEND EXTRA_COMPONENT_DIRS components/esp_lcd_touch components/esp_lcd_touch_gt911 components/pca9557 components/buzzer components/bm8563)
project(ESP32-S3)


