#ifndef __HUMIDITYSENSOR_H__
#define __HUMIDITYSENSOR_H__

#include "pins.h"

class HUMIDITYSENSOR {

  public:
    typedef struct {
      int16_t temp;
      uint16_t adc;
    } characteristics_struct;

    HUMIDITYSENSOR(pins_t *pin_cmoist, pins_t *pin_cpump);
    uint16_t get_humidity(uint8_t relative = 0);
    uint8_t  to_relative(uint16_t value);
    int16_t  interpolate(uint16_t adc, const characteristics_struct *characteristics, uint8_t size);

  private:
    pins_t *pin_cmoist;
    pins_t *pin_cpump;
};

#endif
