/*
## Tasks
* [x] app_main with led_on
* [x] app_main with button led
* [x] move that to a task
* [x] use a queue to trigger the task (with a timer)
* [ ] use ISR to trigger that queue
*/

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_35
#define BUTTON_PIN GPIO_NUM_17
#define BUTTON_TAG "button_check"

QueueHandle_t button_queue;

typedef struct {
    int gpio_num;
} button_event_t;

void task_button_check(void* args) {
    button_event_t event;
    bool prev_state = false;

    while(1) {
        if (xQueueReceive(button_queue, &event, portMAX_DELAY) == pdTRUE) {
            bool button_pressed = gpio_get_level(BUTTON_PIN) == 0;
            if (prev_state != button_pressed) {
                ESP_LOGI(BUTTON_TAG, "Button(%d) state changed: %d", event.gpio_num, button_pressed);
                gpio_set_level(LED_PIN, button_pressed? 1 : 0);
            }
            prev_state = button_pressed;
        }
    }
}

static void IRAM_ATTR gpio_ist_handler(void* args) {
    button_event_t event;
    event.gpio_num = (int)(intptr_t)args;
    BaseType_t higher_task = pdFALSE;
    xQueueSendFromISR(button_queue, &event, &higher_task);
    if (higher_task == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void app_main(void) {
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);

    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // This may trigger multiple times when pressing the button
        // To get a much better result, use GPIO_INTR_NEGEDGE and
        // attach a 100nf capacitor to the pin & the ground
        // It will give us much better accuracy but the POSEDGE is fine for our purpose.
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&button_config);
    gpio_install_isr_service(0);
    // This is a nice hack: (void*)(intptr_t) BUTTON_PIN
    // The ideal way is to pass a pointer as args
    // But since it's just a number, we cast the number into an int pointer and then into (void*)
    // So, we are basically smuggling a integer inside a pointer
    // To get it back, we can do this: (int)(intptr_r)args
    gpio_isr_handler_add(BUTTON_PIN, gpio_ist_handler, (void*)(intptr_t) BUTTON_PIN);

    button_queue = xQueueCreate(10, sizeof(button_event_t));

    xTaskCreate(
        task_button_check,
        "button_check",
        // If we are not doing any debug printing inside the task, we can just use 1024 as the stack depth
        3072,
        NULL,
        5,
        NULL
    );
}