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

#define USER_REG_PRECISION_RH12_TEMP14  0x00
#define USER_REG_PRECISION_RH8_TEMP12   0x01
#define USER_REG_PRECISION_RH10_TEMP13  0x80
#define USER_REG_PRECISION_RH11_TEMP11  0x81

#define WRITE_USER_REG     0xE6
#define READ_USER_REG      0xE7
#define SOFT_RESET         0xFE

class SHT20 {
  public:
    SHT20();
    void     init();
    uint8_t  write_userreg(uint8_t value);
    uint8_t  read_userreg(uint8_t *value);
    void     set_resolution(uint8_t resolution);
    int16_t  get_temperature(uint8_t hold = 0);
    uint16_t get_humidity(uint8_t hold = 0);


  private:
    uint8_t get_data(uint8_t *data, uint8_t reg, uint8_t hold = 0);
};

#endif
