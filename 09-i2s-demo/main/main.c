#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2s_types.h"
#include <math.h>
#include <stdint.h>

#define I2C_SDA_PIN GPIO_NUM_35
#define I2C_SCL_PIN GPIO_NUM_36

#define I2S_MCLK_PIN  GPIO_NUM_4
#define I2S_BCLK_PIN  GPIO_NUM_5
#define I2S_WS_PIN    GPIO_NUM_6
#define I2S_DOUT_PIN  GPIO_NUM_7

#define MCLK 11.2896 // This should be 11.2896 MHz
#define SAMPLE_RATE 44100
#define BLOCK_FRAMES 256

#define TONE_FREQ_HZ 220
#define TONE_SCALE   0.6f   // keep some headroom

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t device;
static i2s_chan_handle_t tx_chan;

esp_err_t i2c_check() {
    uint8_t address = 0;
    uint8_t value = 0;
    return i2c_master_transmit_receive(device, &address, 1, &value, 1, -1);
}

void i2c_init() {
    i2c_master_bus_config_t bus_config = {
        .clk_source  = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    i2c_device_config_t device_config = {
        .device_address = 0x10,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = 400000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &device_config, &device));
    ESP_ERROR_CHECK(i2c_check());
}

uint8_t i2c_read_reg(uint8_t address) {
    uint8_t value = 0;
    i2c_master_transmit_receive(device, &address, 1, &value, 1, -1);
    return value;
}

void i2c_write_reg(uint8_t address, uint8_t value) {
    uint8_t buffer[2] = {address, value};
    i2c_master_transmit(device, buffer, 2, -1);
}

static void i2s_setup(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = BLOCK_FRAMES;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    // This is the configuration for 16bit left justified serial audio
    i2s_std_slot_config_t slot_cfg = {
        .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
        .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
        .slot_mode = I2S_SLOT_MODE_STEREO,
        .slot_mask = I2S_STD_SLOT_LEFT | I2S_STD_SLOT_RIGHT,
        .ws_width = 32,          // WS high (or low) for the entire 16‑bit slot
        .ws_pol = false,         // match ES8388 LRCK polarity; flip if needed
        .bit_shift = false,      // left-justified: MSB changes with LRCK edge
        .left_align = true,
        .big_endian = false,
        .bit_order_lsb = false,
    };

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = SAMPLE_RATE,
            .clk_src = I2S_CLK_SRC_PLL_160M,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,   // 44.1k × 256 = 11.2896 MHz
        },
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    
    std_cfg.slot_cfg.slot_mask      = I2S_STD_SLOT_LEFT | I2S_STD_SLOT_RIGHT;
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.ws_width       = 32;      // WS high for the whole left sample
    std_cfg.slot_cfg.bit_shift      = false;  // left-justified (MSB changes with LRCK edge)
    std_cfg.slot_cfg.left_align     = true;   // already true via macro, kept for clarity

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

static int16_t *tone_cycle = NULL;
static size_t tone_cycle_len = 0;
static size_t tone_cycle_pos = 0;

static void tone_cycle_init(void)
{
    if (tone_cycle) {
        return;
    }
    double samples_per_cycle = (double)SAMPLE_RATE / (double)TONE_FREQ_HZ;
    tone_cycle_len = (size_t)llround(samples_per_cycle);
    if (tone_cycle_len == 0) {
        tone_cycle_len = 1;
    }
    tone_cycle = malloc(tone_cycle_len * sizeof(int16_t));
    if (!tone_cycle) {
        ESP_LOGE("tone", "Failed to allocate %zu-sample tone cycle", tone_cycle_len);
        abort();
    }

    for (size_t i = 0; i < tone_cycle_len; ++i) {
        double phase = (2.0 * M_PI * i) / (double)tone_cycle_len;
        tone_cycle[i] = (int16_t)(sin(phase) * (INT16_MAX * TONE_SCALE));
    }
    tone_cycle_pos = 0;
    ESP_LOGI("tone", "Tone cycle: %zu samples, actual freq %.3f Hz",
             tone_cycle_len, (double)SAMPLE_RATE / (double)tone_cycle_len);
}

static void fill_from_cycle(int32_t *buffer)
{
    for (size_t n = 0; n < BLOCK_FRAMES; ++n) {
        int32_t sample = tone_cycle[tone_cycle_pos++];
        if (tone_cycle_pos == tone_cycle_len) {
            tone_cycle_pos = 0;
        }
        buffer[2 * n]     = 0;
        buffer[2 * n + 1] = sample;
    }
}

static char* render_binary(uint8_t value, char *out)
{
    out[0] = '0';
    out[1] = 'b';
    for (int i = 0; i < 8; ++i) {
        out[2 + i] = (value & (1 << (7 - i))) ? '1' : '0';
    }
    out[10] = '\0';

    return out;
}

void es8388_config() {
    // Set the output volume to -96db
    i2c_write_reg(46, 0b00000000); // R46: LOUT1 volume: need to change to 00011110 for 0db
    i2c_write_reg(47, 0b00000000); // R47: ROUT1 volume: need to change to 00011110 for 0db

    // Power up ES8388 codec
    i2c_write_reg(0, 0b00000110); // R0: set to defaults: 0000 0110
    i2c_write_reg(1, 0b01010000); // R1: ebable analog power: 0101 0000
    i2c_write_reg(2, 0b00000000); // R2: enable chip power: 0000 0000
    i2c_write_reg(3, 0b00000000); // R3: enable ADCs: 0000 1100
    i2c_write_reg(4, 0b00111100); // R4: enable DACs: 0011 1100
    i2c_write_reg(5, 0b00000000); // R5: no low power mode: 0000 0000
    i2c_write_reg(6, 0b00000000); // R6: no low power mode: 0000
    i2c_write_reg(7, 0b01111100); // R7: anaog voltage mgt (default): 0111 1100
    i2c_write_reg(8, 0b10000000); // R8: master mode with auto bclk

    // ADC settings (we will do more later, no needed for this example)
    i2c_write_reg(9, 0b00000000); // R9: mic gain to 0db: 0000 0000
    i2c_write_reg(10, 0b00000000); // Selet inputs
    i2c_write_reg(11, 0b00000010); // Select ADC inputs
    i2c_write_reg(12, 0b00001101); // R12: Set ADC to 16bit, LSBJ format, and enable ADC clock
    i2c_write_reg(13, 0b00000010); // R13: ADC set clocks
    i2c_write_reg(14, 0b00110000);
    i2c_write_reg(15, 0b00100000);
    i2c_write_reg(16, 0b00000000); // R16: ADCL volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(17, 0b00000000); // R17: DACR volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(18, 0b00111000); 
    i2c_write_reg(19, 0b10110000);
    i2c_write_reg(20, 0b00110011);
    i2c_write_reg(21, 0b00000110);
    i2c_write_reg(22, 0b00000000);

    // DAC and output settings
    i2c_write_reg(23, 0b00011010); // R23: 16bit with Left Justified Serial Audio
    i2c_write_reg(24, 0b00000010); // R24: We use single speed with MCLK/256 (for 44.1khz with the defined MCLK)
    i2c_write_reg(26, 0b00000000); // R26: DACL volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(27, 0b00000000); // R27: DACR volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(28, 0b00000000); // R28: some phase inversion and few defaults. set to defaults: 0000 0000
    i2c_write_reg(29, 0b00000000); // 
    i2c_write_reg(38, 0b00000000); // R38: LIN select: 0000 0000 (LIN1 -> LEFT, RIN1 -> RIGHT)
    i2c_write_reg(39, 0b10010000); // R39: need to enable DACL to Mixer and 0db volume 0101 0000
    i2c_write_reg(42, 0b10010000); // R42: need to enable DACR to Mixer and 0db volume 0101 0000
    i2c_write_reg(43, 0b11000000);
    i2c_write_reg(45, 0b00000000);
    i2c_write_reg(48, 0b00000000); // R48: LOUT2 volume: moved to 0000 0000 for -45db
    i2c_write_reg(49, 0b00000000); // R49: ROUT2 volume: moved to 0000 0000 for -45db

    vTaskDelay(pdMS_TO_TICKS(50));

    // Set the output volume to 0db
    i2c_write_reg(46, 0b00011110); // R46: LOUT1 volume: need to change to 00011110 for 0db
    i2c_write_reg(47, 0b00011110); // R47: ROUT1 volume: need to change to 00011110 for 0db
}

void app_main(void)
{
    i2c_init();
    es8388_config();
    i2s_setup();

    tone_cycle_init();

    static int32_t tx_buffer[BLOCK_FRAMES * 2];
    while (true) {
        fill_from_cycle(tx_buffer);
        size_t bytes_written = 0;
        ESP_ERROR_CHECK(i2s_channel_write(tx_chan,
                                          tx_buffer,
                                          sizeof(tx_buffer),
                                          &bytes_written,
                                          portMAX_DELAY));
    }

}
