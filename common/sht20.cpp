#include "sht20.h"
#include "twi.h"
#include "uart.h"

// TODO check checksum when fetching data

SHT20::SHT20() {
}

void SHT20::init() {
  twi_init();
}

uint8_t SHT20::get_data(uint8_t *data, uint8_t reg, uint8_t hold) {
  return twi_read_bytes(SHT20_ADDR, data, reg, 2, !hold); // read temp hold
}

/*
 * returns temperature in 1/100 °C
 * eg 2345 -> 23.45 °C
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
