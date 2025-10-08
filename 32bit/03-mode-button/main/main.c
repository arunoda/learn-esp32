#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_18
#define BUTTON_PIN GPIO_NUM_8

QueueHandle_t button_queue;
bool button_state = false;

static void IRAM_ATTR gpio_button_handler(void* args) {
    uint32_t button_num = 1;
    BaseType_t higherTask;
    xQueueSendFromISR(button_queue, &button_num, &higherTask);
    if (higherTask == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void task_button_check(void* args) {
    uint32_t button_num = 0;
    while(1) {
        BaseType_t received = xQueueReceive(button_queue, &button_num, portMAX_DELAY);
        if (received == pdTRUE) {
            button_state = !button_state;
            ESP_LOGI("button_check", "Button(%d) changed", 10);
            
            gpio_set_level(LED_PIN, button_state);
        }
    }
}

void app_main(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);
    gpio_set_level(LED_PIN, button_state);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        /*
            GPIO_INTR_POSEDGE will trigger when we release the button
            GPIO_INTR_NEGEDGE will trigger when we click it right away
            GPIO_INTR_ANYEDGE will trigger if for both (possible to trigger multiple similar events)
        */
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&button_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_button_handler, (void*)(intptr_t)BUTTON_PIN);

    button_queue = xQueueCreate(10, sizeof(uint32_t));

    xTaskCreate(
        task_button_check,
        "task_button_check",
        4096,
        NULL,
        5,
        NULL
    );
}
