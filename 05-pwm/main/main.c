#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"

#define LED_PIN GPIO_NUM_35
#define LED_PIN_2 GPIO_NUM_36
#define LED_PWM_RES 12
#define LED_PWM_FREQ 1000
#define LED_PWM_MAX ((1 << LED_PWM_RES) - 1)

void app_main(void)
{
    // configure the timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LED_PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LED_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // asign a pin to a channel & a timer
    ledc_channel_config_t channel_conf = {
        .gpio_num = LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf);

    // if we have another pin, we need to assign it to a different channel
    ledc_channel_config_t channel_conf_2 = {
        .gpio_num = LED_PIN_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel_conf_2);

    const int duty_steps = 100;
    const int delay_ms = 10;

    while (1) {
        // upto 0 brighness to max
        for (int i = 0; i <= duty_steps; i++) {
            int duty = (LED_PWM_MAX / duty_steps) * i;
            // set the duty cycle for the channel and update it
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        // from max brightness to zero
        for (int i=duty_steps; i >= 0; i--) {
            int duty = (LED_PWM_MAX / duty_steps) * i;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }

        vTaskDelay(pdMS_TO_TICKS(400));
    }
}
