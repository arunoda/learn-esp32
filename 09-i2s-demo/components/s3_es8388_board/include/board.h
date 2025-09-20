#pragma once

#include "driver/i2c_master.h"
#include "driver/i2s.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct audio_board *audio_board_handle_t;

audio_board_handle_t audio_board_init(void);
void audio_board_deinit(audio_board_handle_t board);

esp_err_t audio_board_codec_start(audio_board_handle_t board);
esp_err_t audio_board_codec_stop(audio_board_handle_t board);
esp_err_t audio_board_set_volume(audio_board_handle_t board, uint8_t value);
const i2s_pin_config_t *audio_board_get_i2s_pins(audio_board_handle_t board);

#ifdef __cplusplus
}
#endif

