#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// ADC_INFO for CV1
#define ADC_UNIT ADC_UNIT_1
#define CV1_ADC_CHANNEL ADC_CHANNEL_8
#define CV2_ADC_CHANNEL ADC_CHANNEL_9

void app_main(void)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc_handle, CV1_ADC_CHANNEL, &channel_config);
    adc_oneshot_config_channel(adc_handle, CV2_ADC_CHANNEL, &channel_config);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int cv1_value = 0;
        adc_oneshot_read(adc_handle, CV1_ADC_CHANNEL, &cv1_value);

        int cv2_value = 0;
        adc_oneshot_read(adc_handle, CV2_ADC_CHANNEL, &cv2_value);
        ESP_LOGI("main", "cv1: %d,\t cv2: %d", cv1_value, cv2_value);
    }
}
