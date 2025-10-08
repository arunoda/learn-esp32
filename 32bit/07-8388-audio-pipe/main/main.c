#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_SDA_PIN             48      // This is CDATA
#define I2C_SCL_PIN             47      // This is CCLK

static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t device;

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
}

esp_err_t i2c_check() {
    uint8_t address = 0;
    uint8_t value = 0;
    return i2c_master_transmit_receive(device, &address, 1, &value, 1, -1);
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

void app_main(void)
{
    i2c_init();
    ESP_ERROR_CHECK(i2c_check());

    // Here we will setup the Audio codec to forward analog line in directly to the line out
    // This is just to test the programming of the codec using registers
    
    // Power up ES8388 codec
    i2c_write_reg(0, 0b00000110); // R0: set to defaults: 0000 0110
    i2c_write_reg(1, 0b01010000); // R1: ebable analog power: 0101 0000
    i2c_write_reg(2, 0b00000000); // R2: enable chip power: 0000 0000
    i2c_write_reg(3, 0b00001100); // R3: enable ADCs: 0000 1100
    i2c_write_reg(4, 0b00111100); // R4: enable DACs: 0011 1100
    i2c_write_reg(5, 0b00000000); // R5: no low power mode: 0000 0000
    i2c_write_reg(6, 0b00000000); // R6: no low power mode: 0000
    i2c_write_reg(7, 0b01111100); // R7: anaog voltage mgt (default): 0111 1100
    i2c_write_reg(8, 0b10000000); // R8: master/slave mode: master mode selected: 1000 0000

    // ADC settings (we will do more later, no needed for this example)
    i2c_write_reg(9, 0b00000000); // R9: mic gain to 0db: 0000 0000
    i2c_write_reg(14, 0b00110000);
    i2c_write_reg(15, 0b00100000);
    i2c_write_reg(16, 0b00000000); // R16: ADCL volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(17, 0b00000000); // R17: ADCR volume, we need to set it to 0000 0000 of 0db
    
    // DAC and output settings
    i2c_write_reg(26, 0b00000000); // R26: DACL volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(27, 0b00000000); // R27: DACR volume, we need to set it to 0000 0000 of 0db
    i2c_write_reg(28, 0b00000000); // R28: some phase inversion and few defaults. set to defaults: 0000 0000
    i2c_write_reg(29, 0b00000000); // 
    i2c_write_reg(38, 0b00000000); // R38: LIN select: 0000 0000 (LIN1 -> LEFT, RIN1 -> RIGHT)
    i2c_write_reg(39, 0b01010000); // R39: need to enable LIN to Mixer and 0db volume 0101 0000
    i2c_write_reg(42, 0b01010000); // R42: need to enable RIN to Mixer and 0db volume 0101 0000
    i2c_write_reg(45, 0b00000000);
    i2c_write_reg(46, 0b00011110); // R46: LOUT1 volume: need to change to 00011110 for 0db
    i2c_write_reg(47, 0b00011110); // R47: ROUT1 volume: need to change to 00011110 for 0db
    i2c_write_reg(48, 0b00000000); // R48: LOUT2 volume: moved to 0000 0000 for -45db
    i2c_write_reg(49, 0b00000000); // R49: ROUT2 volume: moved to 0000 0000 for -45db
}
