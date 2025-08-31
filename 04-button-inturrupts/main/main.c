#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

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
    if (higher_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
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

    // Button ISR
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr, (void *)(intptr_t)BUTTON_PIN);

    // Initiate Queue
    button_queue = xQueueCreate(10, sizeof(button_event_t));

    // Set the initial LED state
    gpio_set_level(LED_PIN, 0);

    while (1) {
        button_event_t event;
        // We get this event when the button state changed.
        // Also, we get this event multiple times too.
        // To change the state, this is the best way to do even if we set it multiple times.
        // We can use a 100nf capacitor from the pin to ground to debounce press side via hardware
        if (xQueueReceive(button_queue, &event, portMAX_DELAY) == pdTRUE) {
            bool button_pressed = gpio_get_level(BUTTON_PIN) == 0;
            printf("Button changed to: %d\n", button_pressed);
            gpio_set_level(LED_PIN, button_pressed);
        }
    }
}
