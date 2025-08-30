#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_17
#define LED_PIN_2 GPIO_NUM_35

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << LED_PIN_2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    while(1) {
        gpio_set_level(LED_PIN, 1);
        gpio_set_level(LED_PIN_2, 1);
        vTaskDelay(200 / portTICK_PERIOD_MS);

        gpio_set_level(LED_PIN, 0);
        gpio_set_level(LED_PIN_2, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
