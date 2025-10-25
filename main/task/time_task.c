
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "time_task.h"

#include "../gui/gui.h"


static const char* TAG = "TIME";

extern SemaphoreHandle_t lvgl_mux;

void time_task(void *pvParameter){

  ESP_LOGI(TAG, "Start time_task");

  for (;;) {

    xSemaphoreTake(lvgl_mux, portMAX_DELAY);
    disp_time();
    xSemaphoreGive(lvgl_mux);

    vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
  }
}