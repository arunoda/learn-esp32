#ifndef ES8388_PLAIN_H_
#define ES8388_PLAIN_H_

// Single-file, C-friendly ES8388 helper for ESP-IDF (new I2C master API).
// - ESP-IDF v5.4 compatible (uses i2c_device_config_t)
// - No Arduino / Wire, no C++
// - Mirrors your original class methods/behavior.

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// ===== Original enums (unchanged) =====
typedef enum { MIXIN1, MIXIN2, MIXRES, MIXADC } mixsel_t;
typedef enum { OUT1, OUT2, OUTALL } outsel_t;
typedef enum { IN1, IN2, IN1DIFF, IN2DIFF } insel_t;
typedef enum { DACOUT, SRCSELOUT, MIXALL } mixercontrol_t;
typedef enum { DISABLE, GENERIC, VOICE, MUSIC } alcmodesel_t;

// ===== ES8388 address & registers =====
#define ES8388_ADDR 0x10
#define ES8388_CONTROL1      0x00
#define ES8388_CONTROL2      0x01
#define ES8388_CHIPPOWER     0x02
#define ES8388_ADCPOWER      0x03
#define ES8388_DACPOWER      0x04
#define ES8388_CHIPLOPOW1    0x05
#define ES8388_CHIPLOPOW2    0x06
#define ES8388_ANAVOLMANAG   0x07
#define ES8388_MASTERMODE    0x08
#define ES8388_ADCCONTROL1   0x09
#define ES8388_ADCCONTROL2   0x0a
#define ES8388_ADCCONTROL3   0x0b
#define ES8388_ADCCONTROL4   0x0c
#define ES8388_ADCCONTROL5   0x0d
#define ES8388_ADCCONTROL6   0x0e
#define ES8388_ADCCONTROL7   0x0f
#define ES8388_ADCCONTROL8   0x10
#define ES8388_ADCCONTROL9   0x11
#define ES8388_ADCCONTROL10  0x12
#define ES8388_ADCCONTROL11  0x13
#define ES8388_ADCCONTROL12  0x14
#define ES8388_ADCCONTROL13  0x15
#define ES8388_ADCCONTROL14  0x16
#define ES8388_DACCONTROL1   0x17
#define ES8388_DACCONTROL2   0x18
#define ES8388_DACCONTROL3   0x19
#define ES8388_DACCONTROL4   0x1a
#define ES8388_DACCONTROL5   0x1b
#define ES8388_DACCONTROL6   0x1c
#define ES8388_DACCONTROL7   0x1d
#define ES8388_DACCONTROL8   0x1e
#define ES8388_DACCONTROL9   0x1f
#define ES8388_DACCONTROL10  0x20
#define ES8388_DACCONTROL11  0x21
#define ES8388_DACCONTROL12  0x22
#define ES8388_DACCONTROL13  0x23
#define ES8388_DACCONTROL14  0x24
#define ES8388_DACCONTROL15  0x25
#define ES8388_DACCONTROL16  0x26
#define ES8388_DACCONTROL17  0x27
#define ES8388_DACCONTROL18  0x28
#define ES8388_DACCONTROL19  0x29
#define ES8388_DACCONTROL20  0x2a
#define ES8388_DACCONTROL21  0x2b
#define ES8388_DACCONTROL22  0x2c
#define ES8388_DACCONTROL23  0x2d
#define ES8388_DACCONTROL24  0x2e
#define ES8388_DACCONTROL25  0x2f
#define ES8388_DACCONTROL26  0x30
#define ES8388_DACCONTROL27  0x31
#define ES8388_DACCONTROL28  0x32
#define ES8388_DACCONTROL29  0x33
#define ES8388_DACCONTROL30  0x34

#ifndef ES8388_I2C_TIMEOUT_MS
#define ES8388_I2C_TIMEOUT_MS 50
#endif

typedef struct {
  i2c_master_bus_handle_t  bus;
  i2c_master_dev_handle_t  dev;
  bool                     bus_owned;
  outsel_t                 outSel;
  insel_t                  inSel;
} es8388_t;

// ===== Low-level I2C helpers =====
static inline esp_err_t es8388_write_reg(es8388_t *es, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  return i2c_master_transmit(es->dev, buf, 2, ES8388_I2C_TIMEOUT_MS);
}
static inline esp_err_t es8388_read_reg(es8388_t *es, uint8_t reg, uint8_t *out_val) {
  return i2c_master_transmit_receive(es->dev, &reg, 1, out_val, 1, ES8388_I2C_TIMEOUT_MS);
}

// ===== Bus/Device setup =====

// Create a NEW master bus and attach ES8388 to it.
static inline esp_err_t es8388_init_new_bus(es8388_t *es,
                                            i2c_port_t port,
                                            gpio_num_t sda,
                                            gpio_num_t scl,
                                            uint32_t   bus_hz) {
  if (!es) return ESP_ERR_INVALID_ARG;
  memset(es, 0, sizeof(*es));

  i2c_master_bus_config_t bus_cfg = {
    .i2c_port = port,
    .sda_io_num = sda,
    .scl_io_num = scl,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .flags = { .enable_internal_pullup = true },
  };
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &es->bus), "ES8388", "bus create failed");

  // v5.4 type & fields
  i2c_device_config_t dev_cfg = {
    .device_address = ES8388_ADDR,
    .scl_speed_hz   = (int)bus_hz,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(es->bus, &dev_cfg, &es->dev), "ES8388", "add dev failed");

  es->bus_owned = true;
  es->outSel = OUTALL;
  es->inSel  = IN1;
  return ESP_OK;
}

// Attach ES8388 to an EXISTING bus.
static inline esp_err_t es8388_attach_on_bus(es8388_t *es,
                                             i2c_master_bus_handle_t bus,
                                             uint32_t dev_speed_hz) {
  if (!es || !bus) return ESP_ERR_INVALID_ARG;
  memset(es, 0, sizeof(*es));
  es->bus = bus;

  i2c_device_config_t dev_cfg = {
    .device_address = ES8388_ADDR,
    .scl_speed_hz   = (int)dev_speed_hz,
  };
  ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &es->dev), "ES8388", "add dev failed");

  es->bus_owned = false;
  es->outSel = OUTALL;
  es->inSel  = IN1;
  return ESP_OK;
}

// Optional: clean up
static inline esp_err_t es8388_deinit(es8388_t *es) {
  if (!es) return ESP_OK;
  if (es->dev) { i2c_master_bus_rm_device(es->dev); es->dev = NULL; }
  if (es->bus_owned && es->bus) { i2c_del_master_bus(es->bus); es->bus = NULL; }
  return ESP_OK;
}

// Probe device address
static inline bool es8388_identify(es8388_t *es) {
  if (!es) return false;
  uint8_t dummy;
  return (es8388_read_reg(es, ES8388_CONTROL1, &dummy) == ESP_OK);
}

// Read all 0..52
static inline esp_err_t es8388_read_all_regs(es8388_t *es, uint8_t *out53) {
  if (!es || !out53) return ESP_ERR_INVALID_ARG;
  for (uint8_t i = 0; i < 53; i++) {
    ESP_RETURN_ON_ERROR(es8388_read_reg(es, i, &out53[i]), "ES8388", "read reg failed");
  }
  return ESP_OK;
}

// ===== High-level (mirrors original methods) =====
static inline esp_err_t es8388_init_sequence(es8388_t *es) {
  esp_err_t err = ESP_OK;
  err |= es8388_write_reg(es, ES8388_MASTERMODE,   0x00);
  err |= es8388_write_reg(es, ES8388_CHIPPOWER,    0xFF);
  err |= es8388_write_reg(es, ES8388_DACCONTROL21, 0x80);
  err |= es8388_write_reg(es, ES8388_CONTROL1,     0x05);
  err |= es8388_write_reg(es, ES8388_CONTROL2,     0x40);

  // ADC
  err |= es8388_write_reg(es, ES8388_ADCPOWER,     0x00);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL2,  0x50);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL3,  0x80);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL1,  0x77);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL4,  0x0C);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL5,  0x02);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL6,  0b00110000);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL8,  0x00);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL9,  0x00);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL10, 0xEA);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL11, 0xC0);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL12, 0x12);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL13, 0x06);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL14, 0xC3);

  // DAC
  err |= es8388_write_reg(es, ES8388_DACPOWER,     0x3C);
  err |= es8388_write_reg(es, ES8388_DACCONTROL1,  0x18);
  err |= es8388_write_reg(es, ES8388_DACCONTROL2,  0x02);
  err |= es8388_write_reg(es, ES8388_DACCONTROL3,  0x00);
  err |= es8388_write_reg(es, ES8388_DACCONTROL4,  0x00);
  err |= es8388_write_reg(es, ES8388_DACCONTROL5,  0x00);
  err |= es8388_write_reg(es, ES8388_DACCONTROL16, 0x09);
  err |= es8388_write_reg(es, ES8388_DACCONTROL17, 0x50);
  err |= es8388_write_reg(es, ES8388_DACCONTROL18, 0x38);
  err |= es8388_write_reg(es, ES8388_DACCONTROL19, 0x38);
  err |= es8388_write_reg(es, ES8388_DACCONTROL20, 0x50);
  err |= es8388_write_reg(es, ES8388_DACCONTROL24, 0x00);
  err |= es8388_write_reg(es, ES8388_DACCONTROL25, 0x00);
  err |= es8388_write_reg(es, ES8388_DACCONTROL26, 0x00);
  err |= es8388_write_reg(es, ES8388_DACCONTROL27, 0x00);

  err |= es8388_write_reg(es, ES8388_CHIPPOWER,    0x00);
  return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t es8388_output_select(es8388_t *es, outsel_t sel) {
  esp_err_t err = ESP_OK;
  if (sel == OUTALL)      err = es8388_write_reg(es, ES8388_DACPOWER, 0x3C);
  else if (sel == OUT1)   err = es8388_write_reg(es, ES8388_DACPOWER, 0x30);
  else /* sel == OUT2 */  err = es8388_write_reg(es, ES8388_DACPOWER, 0x0C);
  if (err == ESP_OK) es->outSel = sel;
  return err;
}

static inline esp_err_t es8388_input_select(es8388_t *es, insel_t sel) {
  esp_err_t err = ESP_OK;
  if (sel == IN1) {
    err = es8388_write_reg(es, ES8388_ADCCONTROL2, 0x00);
  } else if (sel == IN2) {
    err = es8388_write_reg(es, ES8388_ADCCONTROL2, 0x50);
  } else if (sel == IN1DIFF) {
    err |= es8388_write_reg(es, ES8388_ADCCONTROL2, 0xF0);
    err |= es8388_write_reg(es, ES8388_ADCCONTROL3, 0x00);
  } else { // IN2DIFF
    err |= es8388_write_reg(es, ES8388_ADCCONTROL2, 0xF0);
    err |= es8388_write_reg(es, ES8388_ADCCONTROL3, 0x80);
  }
  if (err == ESP_OK) es->inSel = sel;
  return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t es8388_dac_mute(es8388_t *es, bool mute) {
  uint8_t reg = 0;
  if (es8388_read_reg(es, ES8388_ADCCONTROL1, &reg) != ESP_OK) return ESP_FAIL;
  return es8388_write_reg(es, ES8388_DACCONTROL3, mute ? (reg | 0x04) : (reg & ~(0x04)));
}

static inline esp_err_t es8388_set_output_volume(es8388_t *es, uint8_t vol) {
  if (vol > 33) vol = 33;
  esp_err_t err = ESP_OK;
  if (es->outSel == OUTALL || es->outSel == OUT1) {
    err |= es8388_write_reg(es, ES8388_DACCONTROL24, vol);
    err |= es8388_write_reg(es, ES8388_DACCONTROL25, vol);
  }
  if (es->outSel == OUTALL || es->outSel == OUT2) {
    err |= es8388_write_reg(es, ES8388_DACCONTROL26, vol);
    err |= es8388_write_reg(es, ES8388_DACCONTROL27, vol);
  }
  return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t es8388_get_output_volume(es8388_t *es, uint8_t *out_vol) {
  if (!out_vol) return ESP_ERR_INVALID_ARG;
  uint8_t reg = 0;
  esp_err_t err;
  if (es->outSel == OUT2) err = es8388_read_reg(es, ES8388_DACCONTROL26, &reg);
  else                    err = es8388_read_reg(es, ES8388_DACCONTROL24, &reg);
  if (err == ESP_OK) *out_vol = reg;
  return err;
}

static inline esp_err_t es8388_set_input_gain(es8388_t *es, uint8_t gain) {
  if (gain > 8) gain = 8;
  uint8_t v = (uint8_t)((gain << 4) | (gain & 0x0F));
  return es8388_write_reg(es, ES8388_ADCCONTROL1, v);
}

static inline esp_err_t es8388_get_input_gain(es8388_t *es, uint8_t *out_gain) {
  if (!out_gain) return ESP_ERR_INVALID_ARG;
  uint8_t reg = 0;
  esp_err_t err = es8388_read_reg(es, ES8388_ADCCONTROL1, &reg);
  if (err == ESP_OK) *out_gain = (reg & 0x0F);
  return err;
}

static inline esp_err_t es8388_set_alc_mode(es8388_t *es, alcmodesel_t alc) {
  esp_err_t err = ESP_OK;
  uint8_t ALCSEL=0b11, ALCLVL=0b0011, MAXGAIN=0b111, MINGAIN=0b000;
  uint8_t ALCHLD=0b0000, ALCDCY=0b0101, ALCATK=0b0111; // note: ALCDCY=0b0101
  uint8_t ALCMODE=0, ALCZC=0, TIME_OUT=0, NGAT=1, NGTH=0b10001, NGG=0b00, WIN_SIZE=0b00110;

  if (alc == DISABLE) {
    ALCSEL = 0b00;
  } else if (alc == MUSIC) {
    ALCDCY = 0b1010; ALCATK = 0b0110; NGTH = 0b01011;
  } else if (alc == VOICE) {
    ALCLVL = 0b1100; MAXGAIN = 0b101; MINGAIN = 0b010;
    ALCDCY = 0b0001; ALCATK  = 0b0010; NGTH    = 0b11000; NGG = 0b01;
    err |= es8388_write_reg(es, ES8388_ADCCONTROL1, 0x77);
  }

  err |= es8388_write_reg(es, ES8388_ADCCONTROL10, (ALCSEL<<6) | (MAXGAIN<<3) | MINGAIN);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL11, (ALCLVL<<4) | ALCHLD);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL12, (ALCDCY<<4) | ALCATK);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL13, (ALCMODE<<7) | (ALCZC<<6) | (TIME_OUT<<5) | WIN_SIZE);
  err |= es8388_write_reg(es, ES8388_ADCCONTROL14, (NGTH<<3) | (NGG<<2) | NGAT);
  return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t es8388_mixer_source_select(es8388_t *es, mixsel_t LMIXSEL, mixsel_t RMIXSEL) {
  uint8_t reg = (uint8_t)((LMIXSEL << 3) | (RMIXSEL & 0x07));
  return es8388_write_reg(es, ES8388_DACCONTROL16, reg);
}

static inline esp_err_t es8388_mixer_source_control(es8388_t *es,
                                                    bool LD2LO, bool LI2LO, uint8_t LI2LOVOL,
                                                    bool RD2RO, bool RI2RO, uint8_t RI2LOVOL) {
  if (LI2LOVOL > 7) LI2LOVOL = 7;
  if (RI2LOVOL > 7) RI2LOVOL = 7;
  uint8_t regL = ((LD2LO?1:0)<<7) | ((LI2LO?1:0)<<6) | (LI2LOVOL<<3);
  uint8_t regR = ((RD2RO?1:0)<<7) | ((RI2RO?1:0)<<6) | (RI2LOVOL<<3);
  esp_err_t err = ESP_OK;
  err |= es8388_write_reg(es, ES8388_DACCONTROL17, regL);
  err |= es8388_write_reg(es, ES8388_DACCONTROL20, regR);
  return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

static inline esp_err_t es8388_mixer_source_control_combined(es8388_t *es, mixercontrol_t mix) {
  if (mix == DACOUT)     return es8388_mixer_source_control(es, true,  false, 2, true,  false, 2);
  if (mix == SRCSELOUT)  return es8388_mixer_source_control(es, false, true,  2, false, true,  2);
  if (mix == MIXALL)     return es8388_mixer_source_control(es, true,  true,  2, true,  true,  2);
  return ESP_OK;
}

static inline esp_err_t es8388_analog_bypass(es8388_t *es, bool bypass) {
  esp_err_t err = ESP_OK;
  if (bypass) {
    if (es->inSel == IN1) err |= es8388_mixer_source_select(es, MIXIN1, MIXIN1);
    else                  err |= es8388_mixer_source_select(es, MIXIN2, MIXIN2);
    err |= es8388_mixer_source_control(es, false, true, 2, false, true, 2);
  } else {
    err |= es8388_mixer_source_control(es, true,  false, 2, true,  false, 2);
  }
  return (err == ESP_OK) ? ESP_OK : ESP_FAIL;
}

#ifdef __cplusplus
}
#endif
#endif // ES8388_PLAIN_H_
