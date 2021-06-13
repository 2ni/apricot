/*
 * pin configuration settings for forgetmenot
 */

#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "pins.h"
#include "clock.h"

extern CLOCK clock;

class STEPPER {
  public:
    STEPPER();
    void init(pins_t *INA1, pins_t *INA2, pins_t *INB1, pins_t *INB2);
    void set_step(uint8_t step);
    void stop();
    void move_blocking(int16_t steps, uint8_t speed);
    void move(int16_t steps, uint8_t ticks);
    void loop(void (*fn)() = 0);


  private:
    void move_one_step(int8_t direction);

    int8_t current_step;
    int8_t direction;
    uint8_t force_stop;
    int16_t steps_left;
    uint32_t last_move_tick;
    uint8_t speed;

    pins_t *INA1, *INA2, *INB1, *INB2;
};

#endif
