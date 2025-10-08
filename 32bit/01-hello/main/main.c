#include <stdio.h>
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

void app_main(void) {
    printf("Hello world!\n");

    for (int i=0; i < 10; i++) {
        printf("Counting: %d\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("Bye\n");
    fflush(stdout);
    esp_restart();
}