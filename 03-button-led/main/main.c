#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_35
#define BUTTON_PIN GPIO_NUM_17

void app_main(void) {
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_config);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_config);

    bool prev_state = false;

    while(1) {
        // Due to the use of 100ms delay, we don't need to use debouncing
        vTaskDelay(pdMS_TO_TICKS(100));
    
        bool button_pressed = gpio_get_level(BUTTON_PIN) == 0;
        if(prev_state != button_pressed) {
            ESP_LOGI("buttons", "Button state changed: %d\n", button_pressed);
            gpio_set_level(LED_PIN, button_pressed? 1 : 0);
        }
        prev_state = button_pressed;
    }
}