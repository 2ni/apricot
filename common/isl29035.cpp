#include "isl29035.h"
#include "twi.h"
#include "uart.h"
#include <util/delay.h>


ISL29035::ISL29035() {
}

uint8_t ISL29035::init() {
  twi_init();

  uint8_t reg, id;
  reg_read(ISL29035_CHIP, &reg);

  id = (reg >> 3) & 0x07; // b3-b5
  if (id != 0x5) return 1;

  // clear brown out detection bit
  reg_write(ISL29035_CHIP, reg & 0x7f);

  // set default 4000lux, 12bit
  set_ranges(1, 1);

  return 0;
}

uint8_t ISL29035::reg_read(uint8_t reg, uint8_t *data) {
  twi_start(ISL29035_ADDR);
  twi_write(reg);
  twi_stop();
  twi_start(ISL29035_ADDR, 1); // read start

  twi_read(data, 1);
  twi_stop();
  return 0;
}

uint8_t ISL29035::reg_write(uint8_t reg, uint8_t value) {
  twi_start(ISL29035_ADDR);
  twi_write(reg);
  twi_write(value);

  twi_stop();
  return 0;
}

uint8_t ISL29035::read_data(uint16_t *data) {
  uint8_t l, h;

  twi_start(ISL29035_ADDR);
  twi_write(ISL20935_DATA_L);
  twi_stop();

  twi_start(ISL29035_ADDR, 1); // read start
  twi_read(&l, 0);
  twi_read(&h, 1);

  *data = (h<<8) | l;

  return 0;
}

/*
 * see https://en.wikipedia.org/wiki/Daylight:
 * 20000: shade illuminated by entire blue sky, midday
 * 1000-2000: overcast, midday
 *
 * range:
 * 0: 1000  lux
 * 1: 4000  lux
 * 2: 16000 lux
 * 3: 64000 lux
 *
 * 0: 16 bit
 * 1: 12 bit
 * 2:  8 bit
 * 3:  4 bit
 *
 */
uint8_t ISL29035::set_ranges(uint8_t _index_lux, uint8_t _index_adc) {
  if ((index_lux > 3) || (index_adc > 3)) return 1;

  index_lux = _index_lux;
  index_adc = _index_adc;
  return reg_write(ISL29035_CMDII, index_lux | (index_adc << 2));
}

/*
 * index_adc:
 * 0: 16 bit 105ms
 * 1: 12 bit 6.5ms
 * 2:  8 bit 0.41ms
 * 3:  4 bit 0.0256ms
 */
uint8_t ISL29035::measure(uint8_t what, uint16_t *data) {
  reg_write(ISL29035_CMDI, what);
  // TODO use sleep pointer to sleep instead of idling
  if (index_adc > 1) _delay_ms(1);
  else if (index_adc == 1) _delay_ms(7);
  else _delay_ms(105);

  read_data(data);

  return 0;
}

uint8_t ISL29035::raw2lux(uint32_t raw, uint16_t *result) {
  *result = (uint32_t)raw*1000*(1<<(2*index_lux)) / ((uint32_t)1<<((4-index_adc)*4));
  return 0;
}

/*
 * lux = measure * luxrange / 2^adcbits
 *     = measure * 1000*4^index_lux / 2^adcbits
 *     = measure * 1000*2^(2*index_lux) / 2^adcbits
 */
uint8_t ISL29035::read_visible_lux(uint16_t *data) {
  uint16_t raw;
  measure(ISL20935_MODE_ALS_1, &raw);
  // DF("raw: %u, i_lux: %u, i_adc: %u\n", raw, index_lux, index_adc);
  // *data = (uint32_t)raw*1000*(1<<(2*index_lux)) / ((uint32_t)1<<((4-index_adc)*4));

  return raw2lux(raw, data);

  return 0;
}

uint8_t ISL29035::read_ir_lux(uint16_t *data) {
  uint16_t raw;
  measure(ISL20935_MODE_IR_1, &raw);
  *data = (uint32_t)raw*1000*(1<<(2*index_lux)) / ((uint32_t)1<<((4-index_adc)*4));
  return 0;
}

/*
 * see
 * https://github.com/Seeed-Studio/Grove_Digital_Light_Sensor/blob/master/Digital_Light_ISL29035.cpp
 * https://www.renesas.com/eu/en/document/dst/isl29035-datasheet
 *
 */
uint8_t ISL29035::read_ev(uint16_t *data) {
  uint16_t data1, data2;
  measure(ISL20935_MODE_ALS_1, &data1);
  measure(ISL20935_MODE_IR_1, &data2);

  float k = 0.82;
  float beta = -11292.86f;
  if (index_adc > 1) {
      beta = 2137.14f;
  }
 *data = (int32_t)(k * data1 + data2 / beta);
 return 0;
}
