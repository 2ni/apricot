/*
 *
 *      pin_cpump (pin2)
 *       |
 *    -------
 *    ------- Ccharge
 *       |
 *       |- pin_cmoist (pin0)
 *       |
 *     -----
 *     Cmoist
 */

#include <util/delay.h>

#include "humidity_sensor.h"
#include "pins.h"
#include "uart.h"

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

HUMIDITYSENSOR::HUMIDITYSENSOR(pins_t *ipin_cmoist, pins_t *ipin_cpump, pins_t *ipin_temp) {
  pin_cmoist = ipin_cmoist;
  pin_cpump = ipin_cpump;
  pin_temp = ipin_temp;
}

uint16_t HUMIDITYSENSOR::get_humidity(uint8_t relative) {
  uint16_t cycles = 0; // cycles needed to charge Ccharge from Cmoist to threshold of pin_cpump

  // discharge
  pins_output(pin_cmoist, 1);
  pins_set(pin_cmoist, 0);
  pins_output(pin_cpump, 1);
  pins_set(pin_cpump, 0);
  _delay_us(10);

  /*
   * Issue:
   * 1) Ccharge eg has already 1.7v after x cycles
   * 2) We set pin_cpump to 1 to charge Cmoist
   * 3) As Ccharge already has 1.7v, pin_cmoist only reaches 3.3-1.7=1.6v
   * 4) It's not enough to trigger pins_get() -> blocked on while(!pins_get(pin_cmoist))
   * 5) pin_cpump goes high, as never stopped charging with 4)
   * -> charge just for a very short time
   *
   *  pin_cmoist.port_adc->CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp);
   *  pins_getadc(pin_cmoist, VREF_ADC0REFSEL_2V5_gc, 0)
   */
  while (1) {
      // fully charge both caps (volt difference says sth about the size)
    pins_output(pin_cmoist, 0);
    pins_output(pin_cpump, 1);
    pins_set(pin_cpump, 1);
    // charge for a very short time
    // while (!pins_get(pin_cmoist)); // this can fail at some point if
    _NOP();

    // shift load difference to gnd
    pins_output(pin_cpump, 0);
    pins_output(pin_cmoist, 1);
    pins_set(pin_cmoist, 0);

    cycles++;

    // measure if sum > threshold
    if (cycles == 1000 || pins_get(pin_cpump)) break;
  }

  return relative ? to_relative(cycles, characteristics_humidity, characteristics_humidity_len) : cycles;
}

/*
 * get temperature from ntc thermistor
 * if use_float temp is calculated based on Rnominal, Tnominal, beta coefficient, Vbat (expensive)
 * vbat eg 330
 *
 * if !use_float temp based on pre-calculated table based on Vin=3.3v, Vref=1.5v, Rinseries=1M
 * see https://docs.google.com/spreadsheets/d/1KOPVIqWLB8RtdWV2ETXrc3K58mqvxZQhDzQeA-0KVI4/edit#gid=679079026 for data
 */
uint16_t HUMIDITYSENSOR::get_temperature(uint8_t use_float, uint16_t vbat) {
  // get average adc value
  uint8_t samples = 1; // should be factor of 2, eg 2, 4, 8, 16, ...
  uint32_t adc = 0;
  for (uint8_t i=0; i<samples; i++) {
    uint16_t v = pins_getadc(pin_temp, VREF_ADC0REFSEL_1V5_gc);
    // DF("  v: %u\n", v);
    adc += v;
  }
  adc /= samples;

  if (use_float) {
    float vt, t;
    vt = 150.0 * adc / 1023;          // Vref = 1.5v
    t = 1000.0 * vt / (vbat - vt);    // Rseries = 1MΩ
    t /= 100;                       // Rnominal = 100kΩ
    t = log(t);
    t /= 4150;                      // beta coefficient
    t += 1.0 / (25 + 273.15);       // Tnominal = 25°C
    t = 1.0 / t;
    t -= 273.15;

    return (uint16_t)(t*10);
  } else {
    return interpolate(adc, characteristics_temperature, characteristics_temperature_len);
  }
}

uint8_t HUMIDITYSENSOR::to_relative(uint16_t value, const characteristics_struct *characteristics, uint8_t characteristics_len) {
  return (uint8_t)interpolate(value, characteristics, characteristics_len);
}

/*
 * interpolate a value according to a characterstic table
 */
int16_t HUMIDITYSENSOR::interpolate(uint16_t adc, const characteristics_struct *characteristics, uint8_t size) {
  uint8_t match = 255;
  for (uint8_t i=0; i<size; i++) {
    if (adc > characteristics[i].adc) {
      match=i;
      break;
    }
  }

  //  adc outside of characteristics, return min or max
  if (match==0) return characteristics[0].temp;
  else if (match==255) return characteristics[size-1].temp;

  // linear interpolation
  uint16_t temp_start = characteristics[match-1].temp;
  uint16_t temp_end = characteristics[match].temp;
  int16_t adc_start = characteristics[match-1].adc;
  int16_t adc_end = characteristics[match].adc;

  int16_t r = (temp_end-temp_start) * (adc-adc_start);
  int16_t d = adc_end-adc_start;
  r /= d;

  return temp_start+r;
}
