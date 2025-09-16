#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#define LED_PIN GPIO_NUM_35
#define DUTY_RES 12
#define NUM_TIMER LEDC_TIMER_0
#define PWM_SPEED_MODE LEDC_LOW_SPEED_MODE

void app_main() {
    // first we need to configure the timer (timer 0)
    ledc_timer_config_t timer_conf = {
        .timer_num = NUM_TIMER,
        .freq_hz = 1000,
        // this resolution is in bits
        .duty_resolution = DUTY_RES,
        .speed_mode = PWM_SPEED_MODE,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Then we need to assign the LED into the timer via a channel
    ledc_channel_config_t channel_conf = {
        .gpio_num = LED_PIN,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = NUM_TIMER,
        .speed_mode = PWM_SPEED_MODE,
        .intr_type = LEDC_INTR_DISABLE,
        .hpoint = 0,
        .duty = 0
    };
    ledc_channel_config(&channel_conf);

    const int num_iterations = 20;
    const int duty_step = (1 << DUTY_RES) / num_iterations;


    while(1) {
        for (int lc = 0; lc<num_iterations; lc++) {
            int duty = duty_step * lc;

            // This is how we set the duty
            ledc_set_duty(PWM_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(PWM_SPEED_MODE, LEDC_CHANNEL_0);

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}