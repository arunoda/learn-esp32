#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN GPIO_NUM_35 
#define BUTTON_PIN GPIO_NUM_17

typedef struct {
    int gpio_num;
} button_event_t;

static QueueHandle_t button_queue;

static void IRAM_ATTR button_isr(void *arg)
{
    button_event_t event;
    event.gpio_num = (int)(intptr_t)arg;

    BaseType_t higher_task_woken = pdFALSE;
    xQueueSendFromISR(button_queue, &event, &higher_task_woken);

    // When we send an event to the queue, a higher priority task might get triggered
    // Then we need to yeild the CPU to do that by doing a context switch
    // This is it.
    if (higher_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void button_task(void *arg)
{
    for (;;) {
        button_event_t event;
        // We get this event when the button state changed.
        // Also, we get this event multiple times too.
        // To change the state, this is the best way to do even if we set it multiple times.
        // We can use a 100nf capacitor from the pin to ground to debounce press side via hardware
        if (xQueueReceive(button_queue, &event, portMAX_DELAY) == pdTRUE) {
            bool button_pressed = gpio_get_level(BUTTON_PIN) == 0;
            ESP_LOGI("button", "Button changed to: %d\n", button_pressed);
            gpio_set_level(LED_PIN, button_pressed);
        }
    }
}

void app_main(void)
{
    // LED config
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Button config
    gpio_config_t button_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&button_conf);

    // Initiate Queue
    button_queue = xQueueCreate(10, sizeof(button_event_t));

     // Button ISR
     gpio_install_isr_service(0);
     gpio_isr_handler_add(BUTTON_PIN, button_isr, (void *)(intptr_t)BUTTON_PIN);

    // Set the initial LED state
    gpio_set_level(LED_PIN, 0);

    // Run button logic on it's own task
    // For this logic, we need at-least 3072 stack depth, otherwise we are getting stack overflow issues.
    const UBaseType_t button_task_prio = tskIDLE_PRIORITY;
    xTaskCreate(button_task, "button_task", 3072, NULL, button_task_prio, NULL);

}
