#include "board.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdlib.h>

#define I2C_SDA_PIN GPIO_NUM_35
#define I2C_SCL_PIN GPIO_NUM_36

#define I2S_MCLK_PIN GPIO_NUM_4
#define I2S_BCLK_PIN GPIO_NUM_5
#define I2S_WS_PIN   GPIO_NUM_6
#define I2S_DOUT_PIN GPIO_NUM_7

#define ES8388_DEVICE_ADDR 0x10

static const char *TAG = "audio_board";

struct audio_board {
    i2c_master_bus_handle_t i2c_bus;
    i2c_master_dev_handle_t codec;
    i2s_pin_config_t pins;
    bool started;
};

static esp_err_t audio_board_i2c_check(audio_board_handle_t board)
{
    uint8_t address = 0;
    uint8_t value = 0;
    return i2c_master_transmit_receive(board->codec, &address, 1, &value, 1, -1);
}

static esp_err_t audio_board_write_reg(audio_board_handle_t board, uint8_t address, uint8_t value)
{
    uint8_t buffer[2] = {address, value};
    return i2c_master_transmit(board->codec, buffer, sizeof(buffer), -1);
}

audio_board_handle_t audio_board_init(void)
{
    audio_board_handle_t board = calloc(1, sizeof(struct audio_board));
    if (!board) {
        ESP_LOGE(TAG, "Failed to allocate audio board context");
        return NULL;
    }

    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &board->i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(err));
        free(board);
        return NULL;
    }

    const i2c_device_config_t device_config = {
        .device_address = ES8388_DEVICE_ADDR,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = 400000,
    };

    err = i2c_master_bus_add_device(board->i2c_bus, &device_config, &board->codec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add codec device: %s", esp_err_to_name(err));
        i2c_del_master_bus(board->i2c_bus);
        free(board);
        return NULL;
    }

    err = audio_board_i2c_check(board);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Codec not responding on I2C: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(board->codec);
        i2c_del_master_bus(board->i2c_bus);
        free(board);
        return NULL;
    }

    board->pins = (i2s_pin_config_t){
        .mck_io_num = I2S_MCLK_PIN,
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_DOUT_PIN,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };

    ESP_LOGI(TAG, "Audio board initialised");
    return board;
}

void audio_board_deinit(audio_board_handle_t board)
{
    if (!board) {
        return;
    }

    if (board->started) {
        audio_board_codec_stop(board);
    }

    if (board->codec) {
        i2c_master_bus_rm_device(board->codec);
    }
    if (board->i2c_bus) {
        i2c_del_master_bus(board->i2c_bus);
    }
    free(board);
}

const i2s_pin_config_t *audio_board_get_i2s_pins(audio_board_handle_t board)
{
    if (!board) {
        return NULL;
    }
    return &board->pins;
}

static esp_err_t audio_board_configure_codec(audio_board_handle_t board)
{
    ESP_RETURN_ON_FALSE(board, ESP_ERR_INVALID_ARG, TAG, "Invalid board handle");

    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 46, 0b00000000), TAG, "Set initial volume");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 47, 0b00000000), TAG, "Set initial volume");

    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 0, 0b00000110), TAG, "R0");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 1, 0b01010000), TAG, "R1");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 2, 0b00000000), TAG, "R2");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 3, 0b00000000), TAG, "R3");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 4, 0b00111100), TAG, "R4");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 5, 0b00000000), TAG, "R5");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 6, 0b00000000), TAG, "R6");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 7, 0b01111100), TAG, "R7");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 8, 0b10000000), TAG, "R8");

    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 9, 0b00000000), TAG, "R9");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 10, 0b00000000), TAG, "R10");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 11, 0b00000010), TAG, "R11");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 12, 0b00001101), TAG, "R12");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 13, 0b00000010), TAG, "R13");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 14, 0b00110000), TAG, "R14");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 15, 0b00100000), TAG, "R15");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 16, 0b00000000), TAG, "R16");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 17, 0b00000000), TAG, "R17");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 18, 0b00111000), TAG, "R18");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 19, 0b10110000), TAG, "R19");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 20, 0b00110011), TAG, "R20");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 21, 0b00000110), TAG, "R21");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 22, 0b00000000), TAG, "R22");

    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 23, 0b00011010), TAG, "R23");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 24, 0b00000010), TAG, "R24");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 26, 0b00000000), TAG, "R26");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 27, 0b00000000), TAG, "R27");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 28, 0b00000000), TAG, "R28");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 29, 0b00000000), TAG, "R29");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 38, 0b00000000), TAG, "R38");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 39, 0b10010000), TAG, "R39");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 42, 0b10010000), TAG, "R42");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 43, 0b11000000), TAG, "R43");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 45, 0b00000000), TAG, "R45");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 48, 0b00000000), TAG, "R48");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 49, 0b00000000), TAG, "R49");

    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 46, 0b00011110), TAG, "R46");
    ESP_RETURN_ON_ERROR(audio_board_write_reg(board, 47, 0b00011110), TAG, "R47");

    board->started = true;
    return ESP_OK;
}

esp_err_t audio_board_codec_start(audio_board_handle_t board)
{
    return audio_board_configure_codec(board);
}

esp_err_t audio_board_codec_stop(audio_board_handle_t board)
{
    if (!board || !board->started) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = audio_board_write_reg(board, 0, 0b00000000);
    if (err == ESP_OK) {
        board->started = false;
    }
    return err;
}

esp_err_t audio_board_set_volume(audio_board_handle_t board, uint8_t value)
{
    if (!board) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t vol = value;
    if (vol > 0x1F) {
        vol = 0x1F;
    }
    esp_err_t err = audio_board_write_reg(board, 46, vol);
    if (err != ESP_OK) {
        return err;
    }
    return audio_board_write_reg(board, 47, vol);
}

