# Demo application for version 1.3 of CrowPanel Advance 7“ HMI ESP32-S3 AI-Powered IPS Touch Screen with ESP-IDF and LVGL

This is a small demo for the ![CrowPanel Advance 7“ HMI ESP32-S3 AI-Powered IPS Touch Screen]([https://github.com/user-attachments/assets/5b2e9cad-c5c2-48d8-8e11-7e8b55169fdf](https://www.elecrow.com/crowpanel-advance-7-0-hmi-esp32-ai-display-800x480-artificial-intelligent-ips-touch-screen-support-meshtastic-and-arduino-lvgl-micropython.html?idd=5)). 

It uses just LVGL, no other GFX library is needed!

***You can find a detailed explanation [on my website](https://www.haraldkreuzer.net/en/news/crowpanel-advance-7-esp32-basics-and-gui-development-lvgl-esp-idf)***


## This example shows:

- Creating the layouts with EEX-Studio
- Controlling the display and touch controller
- Events in the user interface and corresponding updates to the display
- Changing the display brightness
- Controlling the RTC real-time clock, the IO expander and the buzzer

  
## Project structure:

<pre>
CMakeLists.txt
sdkconfig.defaults                                 Special ESP-IDF settings
components
components\bm8563                                  RTC driver
components\esp_lcd_touch                           General touch driver code
components\esp_lcd_touch_gt911                     Adapted GT911 driver
components\stc8h1k28                               Driver for the IO expander
main
main\CMakeLists.txt
main\app_main.c                                    Main program
main\display
main\display\esp32_s3.c                            Display and LVGL initialization
main\display\esp32_s3.h
main\display\elecrow_advanced_7inch_800x480.h      Header file for the CrowPanel board
main\gui
main\gui\gui.h
main\gui\gui.c                                     Functions for changes in the GUI
main\task
main\task\time_task.h                              Simple task that displays the RTC time
main\task\time_task.c
main\ui                                            This folder contains the EEZ-Studio export
</pre>




![demo_app](https://github.com/user-attachments/assets/5b2e9cad-c5c2-48d8-8e11-7e8b55169fdf)


