#ifndef __TRACKSWITCH_STRUCTS_H__
#define __TRACKSWITCH_STRUCTS_H__
namespace DCC {
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
  } CV_Index;

  typedef struct {
    uint16_t addr;
    uint8_t cv29;
  } CFG;

  typedef struct {
    uint16_t index;
    uint8_t data;
  } CV;

  const uint8_t CV_SIZE = 5;
  const CV cv_defaults[CV_SIZE] = {
    { CV01_ADDR_LSB     , 0x05 },
    { CV09_ADDR_MSB     , 0x01 },
    { CV07_VERSION      , 0x01 },
    { CV08_MANUFACTURER , 0x0d },       // public domain, diy decoders
    { CV29_CONFIG       , 0b11000000 }, // [7]: 1=accessory decoder, [6]: 0=decoder addr, 1=output addr
  };
}
#endif
