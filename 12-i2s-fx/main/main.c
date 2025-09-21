// ESP-IDF (new I2S driver, std mode) — Full-duplex pass-through
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "es8388.h"
#include "hal/gpio_types.h"
#include "esp_adc/adc_oneshot.h"

#define LED_PIN 38
// This is the ADC Unit and Channel for CV1 and CV2
// We need to check the datasheet to find out which is what
#define ADC_UNIT ADC_UNIT_2
#define ADC_CHANNEL_CV1 ADC_CHANNEL_6 // GPIO_17
#define ADC_CHANNEL_CV2 ADC_CHANNEL_5 // GPIO_16

#define BUFF_LEN 64  // samples (uint16_t)
#define SAMPLE_RATE 44100
#define REVERB_COMB1_LENGTH 1499
#define REVERB_COMB2_LENGTH 1733
#define REVERB_COMB3_LENGTH 1949
#define REVERB_ALLPASS1_LENGTH 347
#define REVERB_ALLPASS2_LENGTH 113
#define REVERB_COMB_FEEDBACK 0.9f
#define REVERB_COMB_DAMP 0.25f
#define REVERB_ALLPASS_FEEDBACK 0.64f
#define REVERB_WET_MIX 0.90f
#define MAX_DELAY_TIME_MS 500
#define DELAY_TIME_MS 200
#define DELAY_FEEDBACK 0.6f
#define DELAY_WET_MIX 0.5f
#define DELAY_BUFFER_SAMPLES ((SAMPLE_RATE * DELAY_TIME_MS / 1000) * 2)
#define SAMPLE_TO_FLOAT (1.0f / 32768.0f)
#define FLOAT_TO_SAMPLE 32767.0f

static inline int16_t clamp_int16(int32_t sample)
{
    if (sample > INT16_MAX) {
        return INT16_MAX;
    }
    if (sample < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)sample;
}

static inline float clamp_unit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static i2s_chan_handle_t tx_chan = NULL;
static i2s_chan_handle_t rx_chan = NULL;

static uint16_t rxbuf[BUFF_LEN];
static uint16_t txbuf[BUFF_LEN];

static float cv1 = 0.0f;
static float cv2 = 0.0f;

static void audio_task(void *arg)
{
    size_t n = 0;
    static float comb1[REVERB_COMB1_LENGTH];
    static float comb2[REVERB_COMB2_LENGTH];
    static float comb3[REVERB_COMB3_LENGTH];
    static float comb1_lp = 0.0f;
    static float comb2_lp = 0.0f;
    static float comb3_lp = 0.0f;
    static size_t comb1_idx = 0;
    static size_t comb2_idx = 0;
    static size_t comb3_idx = 0;
    static float allpass1[REVERB_ALLPASS1_LENGTH];
    static size_t allpass1_idx = 0;
    static float allpass2[REVERB_ALLPASS2_LENGTH];
    static size_t allpass2_idx = 0;
    static float delay_buffer[DELAY_BUFFER_SAMPLES];
    static size_t delay_idx = 0;

    // Small delay so clocks settle after enable
    vTaskDelay(pdMS_TO_TICKS(10));

    for (;;) {
        // Read up to BUFF_LEN * 2 bytes (16-bit samples)
        ESP_ERROR_CHECK(i2s_channel_read(rx_chan, rxbuf,
                                         BUFF_LEN * sizeof(uint16_t),
                                         &n, portMAX_DELAY));

        int16_t *input = (int16_t *)rxbuf;
        int16_t *output = (int16_t *)txbuf;
        const size_t samples = n / sizeof(int16_t);

        // Schroeder-style reverb: three combs in parallel, two all-pass in series.
        for (size_t i = 0; i < samples; ++i) {
            const float dry = (float)input[i] * SAMPLE_TO_FLOAT;

            float delayed_sample = delay_buffer[delay_idx];
            float delay_input = clamp_unit(dry + (delayed_sample * DELAY_FEEDBACK * cv1));
            delay_buffer[delay_idx] = delay_input;
            if (++delay_idx >= DELAY_BUFFER_SAMPLES) {
                delay_idx = 0;
            }

            float delay_mix = clamp_unit((dry * (1.0f - DELAY_WET_MIX)) + (delayed_sample * DELAY_WET_MIX));
            const float reverb_in = delay_mix;

            float c1 = comb1[comb1_idx];
            float c2 = comb2[comb2_idx];
            float c3 = comb3[comb3_idx];

            comb1_lp += REVERB_COMB_DAMP * (c1 - comb1_lp);
            comb2_lp += REVERB_COMB_DAMP * (c2 - comb2_lp);
            comb3_lp += REVERB_COMB_DAMP * (c3 - comb3_lp);

            comb1[comb1_idx] = clamp_unit(reverb_in + (comb1_lp * REVERB_COMB_FEEDBACK));
            comb2[comb2_idx] = clamp_unit(reverb_in + (comb2_lp * REVERB_COMB_FEEDBACK));
            comb3[comb3_idx] = clamp_unit(reverb_in + (comb3_lp * REVERB_COMB_FEEDBACK));

            float comb_sum = (comb1_lp + comb2_lp + comb3_lp) * (1.0f / 3.0f);

            if (++comb1_idx >= REVERB_COMB1_LENGTH) {
                comb1_idx = 0;
            }
            if (++comb2_idx >= REVERB_COMB2_LENGTH) {
                comb2_idx = 0;
            }
            if (++comb3_idx >= REVERB_COMB3_LENGTH) {
                comb3_idx = 0;
            }

            float buf1 = comb_sum + (REVERB_ALLPASS_FEEDBACK * allpass1[allpass1_idx]);
            float ap1_out = allpass1[allpass1_idx] - (REVERB_ALLPASS_FEEDBACK * buf1);
            allpass1[allpass1_idx] = clamp_unit(buf1);
            if (++allpass1_idx >= REVERB_ALLPASS1_LENGTH) {
                allpass1_idx = 0;
            }

            float buf2 = ap1_out + (REVERB_ALLPASS_FEEDBACK * allpass2[allpass2_idx]);
            float ap2_out = allpass2[allpass2_idx] - (REVERB_ALLPASS_FEEDBACK * buf2);
            allpass2[allpass2_idx] = clamp_unit(buf2);
            if (++allpass2_idx >= REVERB_ALLPASS2_LENGTH) {
                allpass2_idx = 0;
            }

            float wet = clamp_unit((reverb_in * (1.0f - REVERB_WET_MIX * cv2)) + (ap2_out * REVERB_WET_MIX * cv2));

            int32_t sample = (int32_t)(wet * FLOAT_TO_SAMPLE);
            output[i] = clamp_int16(sample);
        }

        // Or mute: memset(txbuf, 0, n);

        size_t written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx_chan, txbuf, n, &written, portMAX_DELAY));
    }
}

void app_main(void)
{

    // 1) setup es8388 
    es8388_t es = {0};
    // Create a dedicated bus for ES8388 (or use es8388_attach_on_bus if you already have a bus)
    ESP_ERROR_CHECK(es8388_init_new_bus(&es, I2C_NUM_0, GPIO_NUM_35, GPIO_NUM_36, 400000));
    ESP_ERROR_CHECK(es8388_init_sequence(&es));  // same init 

    es8388_output_select(&es, OUT1);
    es8388_input_select(&es, IN1);
    // Here volume 30 is the 0db
    es8388_set_output_volume(&es, 30);
    es8388_set_input_gain(&es, 0);
    es8388_set_alc_mode(&es, DISABLE);
    es8388_mixer_source_select(&es, MIXADC, MIXADC);
    es8388_mixer_source_control_combined(&es, DACOUT);


    // 2 Allocate a full-duplex pair on I2S0 as master
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));  // full-duplex pair
    // (You could pass NULL for one side to do simplex.)

    // 3) Configure std (Philips) mode for BOTH channels
    //    NOTE: In full-duplex std mode, TX and RX should use the same slot/clock cfg. :contentReference[oaicite:1]{index=1}
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = GPIO_NUM_4,   // MCLK out (optional but common for codecs like ES8388)
            .bclk = GPIO_NUM_5,
            .ws   = GPIO_NUM_6,
            .dout = GPIO_NUM_7,
            .din  = GPIO_NUM_15,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    // 4) Enable both channels (starts clocks; MCLK begins after init; BCLK/WS after enable). :contentReference[oaicite:2]{index=2}
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    // 5) Run audio loop in core 1
    xTaskCreatePinnedToCore(audio_task, "audio_task", 4096, NULL, 5, NULL, APP_CPU_NUM);

    // 6) LED setup (for some blocking task for the core 0)
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);

    // 7) CV1 Read
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
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_CV1, &channel_config);
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_CV2, &channel_config);

    while(1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));

        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));

        int read_value = 0;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_CV1, &read_value);
        cv1 = read_value / 4095.0f;
        ESP_LOGI("cv1", "value: %f", cv1);

        adc_oneshot_read(adc_handle, ADC_CHANNEL_CV2, &read_value);
        cv2 = read_value / 4095.0f;
        ESP_LOGI("cv2", "value: %f", cv2);
    }
}
