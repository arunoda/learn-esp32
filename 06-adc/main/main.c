#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// This is the ADC Unit and Channel for GPIO_17
// We need to check the datasheet to find out which is what
#define ADC_UNIT ADC_UNIT_2
#define ADC_CHANNEL ADC_CHANNEL_6

void app_main(void)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        // We need more attenuation to capture 3.3V. 
        // So, we need to add 1.5k (1k + 560) resistors before the 3.3v
        // Then the whole 50k knob range is functional (0 - 4095)
        .atten = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &channel_config);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int read_value = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL, &read_value);
        ESP_LOGI("main", "value: %d", read_value);
    }
}
