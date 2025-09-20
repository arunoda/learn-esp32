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


#define LED_PIN 17
#define BUFF_LEN 32  // samples (uint16_t)
#define SAMPLE_RATE 44100
#define DELAY_TIME_MS 200
#define DELAY_CHANNELS 2
#define DELAY_BUFFER_SAMPLES ((SAMPLE_RATE * DELAY_TIME_MS / 1000) * DELAY_CHANNELS)

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

static i2s_chan_handle_t tx_chan = NULL;
static i2s_chan_handle_t rx_chan = NULL;

static uint16_t rxbuf[BUFF_LEN];
static uint16_t txbuf[BUFF_LEN];

static void audio_task(void *arg)
{
    size_t n = 0;
    static int16_t delay_line[DELAY_BUFFER_SAMPLES];
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
        size_t idx = delay_idx;

        for (size_t i = 0; i < samples; ++i) {
            const int16_t dry = input[i];
            const int16_t delayed = delay_line[idx];
            const int32_t wet = dry + (delayed >> 1);
            output[i] = clamp_int16(wet);

            const int32_t feedback = dry + (delayed >> 2);
            delay_line[idx] = clamp_int16(feedback);

            if (++idx >= DELAY_BUFFER_SAMPLES) {
                idx = 0;
            }
        }

        delay_idx = idx;

        // Or mute: memset(txbuf, 0, n);

        size_t written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx_chan, txbuf, n, &written, portMAX_DELAY));

        taskYIELD();
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

    while(1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));

        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
