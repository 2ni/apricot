/*
 * pin configuration settings for forgetmenot
 */

#ifndef __ISL29035_H__
#define __ISL29035_H__

#include "pins.h"

#define ISL29035_ADDR        0x44
#define ISL29035_CHIP        0x0f
#define ISL29035_CMDI        0x00
#define ISL29035_CMDII       0x01
#define ISL20935_DATA_L      0x02
#define ISL20935_DATA_H      0x03
#define ISL20935_INT_LT_L    0x04
#define ISL20935_INT_LT_H    0x05
#define ISL20935_INT_HT_L    0x06
#define ISL20935_INT_HT_H    0x07

#define ISL20935_MODE_ALS_1 ((0x1)<<5) // once
#define ISL20935_MODE_IR_1  ((0x2)<<5)
#define ISL20935_MODE_ALS_C ((0x5)<<5) // continuous
#define ISL20935_MODE_IR_C  ((0x6)<<5)

class ISL29035 {
  public:
    ISL29035();
    uint8_t  init();
    uint8_t  reg_read(uint8_t reg, uint8_t *data);
    uint8_t  reg_write(uint8_t reg, uint8_t value);
    uint8_t  read_data(uint16_t *data);
    uint8_t  set_ranges(uint8_t _index_lux, uint8_t _index_cycle);

    uint8_t  read_visible_lux(uint16_t *data);
    uint8_t read_ir_lux(uint16_t *data);
    uint8_t read_ev(uint16_t *data);

  private:
    uint8_t index_lux = 1; // 0-3 1000*4^index_range : 1000 - 64000
    uint8_t index_adc = 1; // 0-3 (4-index_adc)*4  : 16, 12, 8, 4

    uint8_t raw2lux(uint32_t raw, uint16_t *result);
    uint8_t measure(uint8_t what, uint16_t *data);
};

#endif
