#include <stdio.h>

#include "esp_err.h"
#include "esp_log.h"

#include "display/esp32_s3.h"
#include "ui/ui.h"
#include "task/time_task.h"



static const char* TAG = "MAIN";

extern SemaphoreHandle_t lvgl_mux;


void app_main(void)
{

    init_display();
    
    ESP_LOGI(TAG, "Start LVGL");

    if (lvgl_port_lock(-1)) {
        ui_init();
        lvgl_port_unlock();
    }

    xTaskCreatePinnedToCore(
      time_task,      /* Task function. */
      "Time Task",    /* String with name of task. */
      4096,           /* Stack size in bytes. */
      NULL,           /* Parameter passed as input of the task */
      0,              /* Priority of the task. */
      NULL,           /* Task handle. */
      1);             /* Clock task on core 1 */

}