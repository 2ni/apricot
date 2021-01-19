#include "sht20.h"
#include "twi.h"
#include "uart.h"


SHT20::SHT20() {
}

void SHT20::init() {
  twi_init();
}

/*
 * set precision (temp/hum in bits)
 * 14/12, 12/8, 11/11, 13/10
 */
uint8_t SHT20::write_userreg(uint8_t value) {
  uint8_t status;
  status = twi_start(SHT20_ADDR);
  if (status != 0) return status;

  status = twi_write(WRITE_USER_REG);
  if (status != 0) return status;

  status = twi_write(value);
  if (status != 0) return status;

  twi_stop();
  return 0;
}

uint8_t SHT20::read_userreg(uint8_t *data) {
  uint8_t status;
  status = twi_start(SHT20_ADDR); // write start
  if (status != 0) return status;

  status = twi_write(READ_USER_REG);
  if (status != 0) return status;

  twi_stop();

  status = twi_start(SHT20_ADDR, 1); // read start
  if (status != 0) return status;

  status = twi_read(data, 1);
  if (status != 0) return status;

  twi_stop();
  return 0;
}

void SHT20::set_resolution(uint8_t resolution) {
  uint8_t reg;
  read_userreg(&reg);
  reg &= 0b01111110; // reset resolution bits (see datasheet p.9
  resolution &= 0b10000001;
  reg |= resolution;
  write_userreg(reg);
}

/*
 * TODO check checksum when fetching data
 * see https://github.com/DFRobot/DFRobot_SHT20/blob/master/DFRobot_SHT20.cpp
 * and https://github.com/technoblogy/tiny-mega-i2c/blob/master/TinyMegaI2CMaster.cpp
 */
uint8_t SHT20::get_data(uint8_t *data, uint8_t reg, uint8_t hold) {
  return twi_read_bytes(SHT20_ADDR, data, reg, 2, !hold); // read temp hold
}

/*
 * returns temperature in 1/100 °C
 * eg 2345 -> 23.45 °C
 * default precision: 14bit
 */
int16_t SHT20::get_temperature(uint8_t hold) {
  uint8_t data[3] = {0};
  uint32_t raw;
  uint8_t status;

  if (hold) {
    status = get_data(data, SHT20_TEMP_HOLD, 1); // read temp hold
  } else {
    status = get_data(data, SHT20_TEMP_NOHOLD); // read temp no hold
  }

  if (status == 0) {
    raw = ((uint32_t)data[0]<<8) | (uint32_t)data[1];
    // DF("%u %u\n", data[0], data[1]);
    // DF("temp raw: %lu\n", raw_temp);
    raw *= 17572;
    raw /= 65536;
    raw -= 4685;

    return (int16_t)raw;
  }
  return 0x8000;
}

/*
 * get humidity in 1/100 %
 * eg 2345 -> 23.45%
 * default precision: 12bit
 */
uint16_t SHT20::get_humidity(uint8_t hold) {
  uint8_t data[3] = {0};
  uint32_t raw;
  uint8_t status;

  if (hold) {
    status = get_data(data, SHT20_HUMID_HOLD, 1); // read humidity hold
  } else {
    status = get_data(data, SHT20_HUMID_NOHOLD); // read humidity no hold
  }

  if (status == 0) {
    raw = ((uint32_t)data[0]<<8) | (uint32_t)data[1];
    raw *= 12500;
    raw /= 65536;
    raw -= 6;

    return (uint16_t)raw;
  }

  return 0x8000;
}
