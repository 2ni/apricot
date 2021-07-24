/*
 *
 * pin config of sensor:
 *
 *     |----
 * ----| Temp
 * ----| Cmoist
 * ----| Cpump
 *   --| GND
 *     |----
 */
#ifndef __HUMIDITYSENSOR_H__
#define __HUMIDITYSENSOR_H__

#include "pins.h"

class HUMIDITYSENSOR {

  private:
    pins_t *pin_cmoist;
    pins_t *pin_cpump;
    pins_t *pin_temp;

  public:
    typedef struct {
      int16_t temp;
      uint16_t adc;
    } characteristics_struct;

    HUMIDITYSENSOR(pins_t *pin_cmoist, pins_t *pin_cpump, pins_t *pin_temp);
    uint16_t get_humidity(uint8_t relative = 0);
    uint16_t get_temperature(uint8_t use_float = 0, uint16_t vbat = 330);
    uint8_t  to_relative(uint16_t value, const characteristics_struct *characteristics, uint8_t characteristics_len);
    int16_t  interpolate(uint16_t adc, const characteristics_struct *characteristics, uint8_t size);

   /*
     * Ccharge = 100nF:
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
    const static uint8_t characteristics_humidity_len = 9;
    const characteristics_struct characteristics_humidity[characteristics_humidity_len] = {
      {0, 490},
      {12, 470},
      {25, 290},
      {37, 160},
      {50, 120},
      {63, 90},
      {75, 70},
      {88, 57},
      {100, 50}
    };

    /*
     * see https://docs.google.com/spreadsheets/d/1KOPVIqWLB8RtdWV2ETXrc3K58mqvxZQhDzQeA-0KVI4/edit#gid=679079026
     * based on Vcc=3.3v, Vref=1.5v, Rinseries=1M
     *
     * +3.3--|1M|--[ntc]--|
     *
     */
    const static uint8_t characteristics_temperature_len = 14;
    const characteristics_struct characteristics_temperature[characteristics_temperature_len] = {
      {-145, 1023},
      {-100, 876},
      {-50,  724},
      {0,    593},
      {50,   481},
      {100,  389},
      {150,  314},
      {200,  253},
      {250,  205},
      {300,  166},
      {350,  135},
      {400,  110},
      {450,  90},
      {500,  74}
    };
};

#endif
