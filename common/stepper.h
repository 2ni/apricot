/*
 * pin configuration settings for forgetmenot
 */

#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "pins.h"

class STEPPER {
  public:
    STEPPER();
    void init(pins_t *INA1, pins_t *INA2, pins_t *INB1, pins_t *INB2);
    void set_step(uint8_t step);
    void stop();
    void move_one_step(int8_t direction);
    void move(int16_t steps, uint8_t speed);


  private:
    int8_t current_step;
    int8_t direction;
    pins_t *INA1, *INA2, *INB1, *INB2;
};

#endif
