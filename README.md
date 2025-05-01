# Demo application for CrowPanel Advance 7“ HMI ESP32-S3 AI-Powered IPS Touch Screen with ESP-IDF and LVGL

This is a small demo for the ![CrowPanel Advance 7“ HMI ESP32-S3 AI-Powered IPS Touch Screen]([https://github.com/user-attachments/assets/5b2e9cad-c5c2-48d8-8e11-7e8b55169fdf](https://www.elecrow.com/crowpanel-advance-7-0-hmi-esp32-ai-display-800x480-artificial-intelligent-ips-touch-screen-support-meshtastic-and-arduino-lvgl-micropython.html?idd=5)). 

It uses just LVGL, no other GFX library is needed!

***You can find a detailed explanation [on my website](https://www.haraldkreuzer.net/en/news/crowpanel-advance-7-esp32-basics-and-gui-development-lvgl-esp-idf)***


## This example shows:

- Creating the layouts with SquareLine Studio
- Controlling the display and touch controller
- Events in the user interface and corresponding updates to the display
- Changing the display from a task
- Controlling the RTC real-time clock, the IO expander and the buzzer

  
## Project structure:

<pre>
CMakeLists.txt
platformio.ini                                     Configuration of the project
sdkconfig.defaults                                 Special ESP-IDF settings
boards\crowpanel advance_s3.json                   Configuration of the CrowPanel board
components
components\bm8563                                  RTC driver
components\buzzer                                  Simple driver for the buzzer
components\esp_lcd_touch                           General touch driver code
components\esp_lcd_touch_gt911                     Adapted GT911 driver
components\pca9557                                 Driver for the IO expander
src
src\CMakeLists.txt
src\main.c                                         Main program
src\lv_conf.h                                      LVGL configuration
src\display
src\display\esp32_s3.c                             Display and LVGL initialization
src\display\esp32_s3.h
src\display\elecrow_advanced_7inch_800x480.h       Header file for the CrowPanel board
src\gui
src\gui\gui.h
src\gui\gui.c                                      Functions for changes in the GUI
src\task
src\task\time_task.h                               Simple task that displays the RTC time
src\task\time_task.c
src\ui                                             This folder contains the Squareline Studio export
</pre>




![demo_app](https://github.com/user-attachments/assets/5b2e9cad-c5c2-48d8-8e11-7e8b55169fdf)


