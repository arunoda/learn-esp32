#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"

// ADC_INFO for CV1 and CV2
#define ADC_UNIT ADC_UNIT_1
#define CV1_ADC_CHANNEL ADC_CHANNEL_8
#define CV2_ADC_CHANNEL ADC_CHANNEL_9

// Continuous ADC settings
#define CONVERSION_FRAME_SIZE 100 // Buffer size in bytes for each frame
#define SAMPLE_FREQ_HZ 20000 // Sample rate in Hz

// Semaphore for data notification
static SemaphoreHandle_t adc_semaphore;
static adc_continuous_handle_t adc_handle;

// The buffer where DMA stores the ADC results
static uint8_t result_buf[CONVERSION_FRAME_SIZE];

// The callback function is triggered from the ISR
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t* edata, void* user_data) {
    BaseType_t must_yield = pdFALSE;
    xSemaphoreGiveFromISR(adc_semaphore, &must_yield);
    return (must_yield == pdTRUE);
}

// The task that processes the ADC data
static void adc_task(void* arg) {
    while (1) {
        // Wait for the ISR to signal that new data is ready
        xSemaphoreTake(adc_semaphore, portMAX_DELAY);

        uint32_t out_len = 0;
        esp_err_t ret = adc_continuous_read(adc_handle, result_buf, CONVERSION_FRAME_SIZE, &out_len, 0);
        if (ret == ESP_OK) {
            for (int i = 0; i < out_len; i += 4) {
                adc_digi_output_data_t *p_data = (adc_digi_output_data_t*)&result_buf[i];
                uint32_t raw_data = p_data->raw_data;
                uint32_t channel = p_data->channel;

                if (channel == CV1_ADC_CHANNEL) {
                    ESP_LOGI("adc_task", "CV1 value: %d", raw_data);
                } else if (channel == CV2_ADC_CHANNEL) {
                    ESP_LOGI("adc_task", "CV2 value: %d", raw_data);
                }
            }
        }
    }
}

void app_main(void) {
    adc_semaphore = xSemaphoreCreateBinary();

    // 1. Configure and install continuous ADC driver
    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = CONVERSION_FRAME_SIZE,
        .conv_frame_size = CONVERSION_FRAME_SIZE,
    };
    adc_continuous_new_handle(&handle_cfg, &adc_handle);

    // 2. Configure channels and sampling frequency
    adc_channel_t channels[] = {CV1_ADC_CHANNEL, CV2_ADC_CHANNEL};
    adc_continuous_config_t config = {
        .pattern_num = sizeof(channels) / sizeof(adc_channel_t),
        .adc_pattern = {
            {.atten = ADC_ATTEN_DB_12, .channel = CV1_ADC_CHANNEL},
            {.atten = ADC_ATTEN_DB_12, .channel = CV2_ADC_CHANNEL},
        },
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    adc_continuous_config(adc_handle, &config);

    // 3. Register callback for when a conversion frame is ready
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);

    // 4. Start the continuous conversion
    adc_continuous_start(adc_handle);

    // 5. Create the task to process data
    xTaskCreate(adc_task, "adc_task", 4096, NULL, 5, NULL);
}

