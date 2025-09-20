#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

void app_main(void)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI("main", "Hello World!");
    }
}
