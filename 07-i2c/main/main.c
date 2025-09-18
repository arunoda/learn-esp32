#include "esp_log.h"
#include "driver/i2c_master.h"

// This is the i2c driver for ES8388 Audio Codec

#define I2C_SDA_PIN             35      // This is CDATA
#define I2C_SCL_PIN             36      // This is CCLK
#define I2C_PORT_NUM            0       // I2C controller 0
#define I2C_FREQ_HZ             400000
#define I2C_ADDR                0x10    // This is the address when the CE is grounded.

static const char *TAG = "i2c-es8388";

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t device;

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,   // still recommended to add 4.7k externals
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address  = I2C_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &device));
}

static esp_err_t i2c_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(device, buf, sizeof(buf), -1);
}

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *val)
{
    // Repeated-start: write reg, then read 1 byte
    return i2c_master_transmit_receive(device, &reg, 1, val, 1, -1);
}

static void render_binary(uint8_t value, char *out)
{
    out[0] = '0';
    out[1] = 'b';
    for (int i = 0; i < 8; ++i) {
        out[2 + i] = (value & (1 << (7 - i))) ? '1' : '0';
    }
    out[10] = '\0';
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C for ES8388");
    i2c_init();

    ESP_ERROR_CHECK(i2c_write_reg(0, 0b00000110));

    for (uint8_t reg = 0; reg < 48; ++reg) {
        uint8_t value = 0;
        ESP_ERROR_CHECK(i2c_read_reg(reg, &value));
        char bits[11];
        render_binary(value, bits);
        ESP_LOGI(TAG, "Reg %d = %s", reg, bits);
    }
}
