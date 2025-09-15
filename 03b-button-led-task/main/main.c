#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_35
#define BUTTON_PIN GPIO_NUM_17


void task_check_button_state(void* args) {
    bool prev_button_state = false;
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(30));
        bool button_pressed = gpio_get_level(BUTTON_PIN) == 0;
        
        if (button_pressed != prev_button_state) {
            ESP_LOGI("buttons", "Button Pressed: %d", button_pressed);
            gpio_set_level(LED_PIN, button_pressed? 1: 0);
        }

        prev_button_state = button_pressed;
    }
}

void app_main(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
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

    xTaskCreate(
        task_check_button_state,
        "CheckButtonState",
        2048,
        NULL,
        5,
        NULL
    );

    // Now there's no reason to run the main as the button handling is moved to a task
}
