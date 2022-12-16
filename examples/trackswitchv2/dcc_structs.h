#ifndef __TRACKSWITCH_STRUCTS_H__
#define __TRACKSWITCH_STRUCTS_H__

namespace DCC {
  enum DEFAULTS {
    CV08_MANUFACTURER_DIY = 0x0d, // https://www.nmra.org/sites/default/files/standards/sandrp/pdf/appendix_a_s-9_2_2_5.pdf
    DECODER_ADDR = 261, // 0x0105
  };

  typedef enum {
    MOTOR_FWD,
    MOTOR_STOP,
    MOTOR_REW,
  } MOTOR;

  typedef struct {
    uint8_t preamble;
    uint8_t data[6];
    uint8_t len;
  } PACKET;

  typedef enum {
    STATE_PREAMBLE,
    STATE_START_BIT,
    STATE_DATA_BYTE,
    STATE_END_BIT,
  } STATE;

  typedef enum {
    SERVICE = 0,
    OPS     = 1,
  } MODE;

  typedef enum {
    CV01_ADDR_LSB,
    CV07_VERSION,
    CV08_MANUFACTURER,
    CV09_ADDR_MSB,
    CV29_CONFIG,
    CV33_POSITION,
    CV34_DELAY,
  } CV_Index;

  typedef struct {
    uint16_t addr;
    uint8_t cv29;
    uint8_t current_position;
    uint16_t delay;
  } CFG;

  typedef struct {
    uint16_t index;
    uint8_t data;
  } CV;

  // https://dccwiki.com/Configuration_Variable
  const uint8_t CV_SIZE = 7;
  const CV cv_defaults[CV_SIZE] = {
    { CV01_ADDR_LSB     , DCC::DECODER_ADDR & 0xff },
    { CV09_ADDR_MSB     , (DCC::DECODER_ADDR >> 8) & 0xff },
    { CV07_VERSION      , 0x01 },
    { CV08_MANUFACTURER , DCC::CV08_MANUFACTURER_DIY },
    { CV29_CONFIG       , 0b11000000 }, // [7]: 1=accessory decoder, [6]: 0=decoder addr, 1=output addr
    { CV33_POSITION     , 0 },
    { CV34_DELAY        , 50 }, // 20ms steps
  };
}
#endif
