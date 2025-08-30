#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN GPIO_NUM_35
#define BUTTON_PIN GPIO_NUM_17

void app_main(void)
{
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
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button_config);

    bool prev_state = false;
    int counter = 0;

    gpio_set_level(LED_PIN, 0);

    while(1) {
        // When we set 1ms polling, debounce is not working
        // most probably because this is not waiting for the 1ms
        // this 10ms is fixing the above
        vTaskDelay(pdMS_TO_TICKS(10));

        bool button_pressed = gpio_get_level(BUTTON_PIN) == 0;
        if (button_pressed != prev_state) {
            counter++;
            
            // debounce of 300 ms
            if (counter > 30) {
                
                counter = 0;
                prev_state = button_pressed;

                if (button_pressed) {
                    gpio_set_level(LED_PIN, 1);
                } else  {
                    gpio_set_level(LED_PIN, 0);
                }
            }
        }
    }
}
