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

HUMIDITYSENSOR::HUMIDITYSENSOR(pins_t *ipin_cmoist, pins_t *ipin_cpump) {
  pin_cmoist = ipin_cmoist;
  pin_cpump = ipin_cpump;
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
   *
   * -> try using adc: will be too slow and caps both directly fully charged
   *  pin_cmoist.port_adc->CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp);
   *  pins_getadc(pin_cmoist, VREF_ADC0REFSEL_2V5_gc, 0)
   */
  while (1) {
      // fully charge both caps (volt difference says sth about the size)
    pins_output(pin_cmoist, 0);
    pins_output(pin_cpump, 1);
    pins_set(pin_cpump, 1);
    // charge for a very short time
    // while (!pins_get(pin_cmoist));
    _NOP();

    // shift load difference to gnd
    pins_output(pin_cpump, 0);
    pins_output(pin_cmoist, 1);
    pins_set(pin_cmoist, 0);

    cycles++;

    // measure if sum > threshold
    if (cycles == 1000 || pins_get(pin_cpump)) break;
  }

  return relative ? to_relative(cycles) : cycles;
}

/*
 * Ccharge = 200nF:
 * air:  2000
 * 12%:  500
 * 25%:  350
 * 37%:  200
 * 50%:  160
 * 63%.  120
 * 75%:  110
 * 88%:  95
 * 100%: 85
 */
uint8_t HUMIDITYSENSOR::to_relative(uint16_t value) {
  const characteristics_struct humidity_characteristics[] = {
    {0, 1000},
    {12, 500},
    {25, 350},
    {37, 200},
    {50, 160},
    {63, 120},
    {75, 110},
    {88, 95},
    {100, 85}
  };

  return (uint8_t)interpolate(value, humidity_characteristics, 9);
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
