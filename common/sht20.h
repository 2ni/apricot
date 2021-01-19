/*
 * pin configuration settings for forgetmenot
 */

#ifndef __SHT20_H__
#define __SHT20_H__

#include "pins.h"

#define SHT20_ADDR         0x40
#define SHT20_TEMP_HOLD    0xE3
#define SHT20_HUMID_HOLD   0xE5
#define SHT20_TEMP_NOHOLD  0xF3
#define SHT20_HUMID_NOHOLD 0xF5
class SHT20 {
  public:
    SHT20();
    int16_t  get_temperature(uint8_t hold = 0);
    uint16_t get_humidity(uint8_t hold = 0);

    void init();

  private:
    uint8_t get_data(uint8_t *data, uint8_t reg, uint8_t hold = 0);
};

#endif
