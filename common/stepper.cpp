#include <util/delay.h>
#include "stepper.h"
#include "pins.h"

STEPPER::STEPPER() {
}

void STEPPER::init(pins_t *iINA1, pins_t *iINA2, pins_t *iINB1, pins_t *iINB2) {
  INA1 = iINA1;
  INA2 = iINA2;
  INB1 = iINB1;
  INB2 = iINB2;
  pins_output(INA1, 1);
  pins_output(INA2, 1);
  pins_output(INB1, 1);
  pins_output(INB2, 1);

  direction = 0;
  current_step = 0;
  steps_left = 0;
}

void STEPPER::set_step(uint8_t step) {
  switch (step) {
    case 0: // 1010
      pins_set(INA1, 1);
      pins_set(INA2, 0);
      pins_set(INB1, 1);
      pins_set(INB2, 0);
      break;
    case 1: // 0110
      pins_set(INA1, 0);
      pins_set(INA2, 1);
      pins_set(INB1, 1);
      pins_set(INB2, 0);
      break;
    case 2: // 0101
      pins_set(INA1, 0);
      pins_set(INA2, 1);
      pins_set(INB1, 0);
      pins_set(INB2, 1);
      break;
    case 3: // 1001
      pins_set(INA1, 1);
      pins_set(INA2, 0);
      pins_set(INB1, 0);
      pins_set(INB2, 1);
      break;
  }
}

void STEPPER::stop() {
  pins_set(INA1, 0);
  pins_set(INA2, 0);
  pins_set(INB1, 0);
  pins_set(INB2, 0);
  force_stop = 1;
  steps_left = 0;
}

/*
 * activates motor coil without moving
 * used eg to keep position with power or
 * eg just to consume some power for ack on DCC signals
 */
void STEPPER::keep() {
  set_step(current_step);
}

void STEPPER::move_one_step(int8_t direction) {
  current_step += direction;
  if (direction == 1 && current_step == 4) current_step = 0;
  if (direction == -1 && current_step == -1) current_step = 3;

  set_step(current_step);
}

/*
 * if stop() called in ISR and stepper is on th move
 * it'll set force_stop = 1 and stop it
 */
void STEPPER::move_blocking(int16_t steps, uint8_t speed) {
  force_stop = 0;
  uint16_t steps_left = steps >= 0 ? steps: -steps;
  while (steps_left && !force_stop) {
    move_one_step(steps >= 0 ? 1 : -1);
    steps_left--;
    _delay_ms(1);
  }
  stop();
}

/*
 * speed is given in number of ticks,
 * ie amount of time to wait between the moves
 * eg delay 1ms with 1 tick = 1/32768 = 30.6us -> set speed = 30
 */
void STEPPER::move(int16_t steps, uint8_t speed) {
  steps_left = steps;
  this->speed = speed;
  last_move_tick = clock.current_tick;
}

void STEPPER::loop(void (*fn)()) {
  // move as long as steps_left <> 0
  if (steps_left && (clock.current_tick - last_move_tick) > speed) {
    last_move_tick = clock.current_tick;
    move_one_step(steps_left > 0 ? 1 : -1);
    steps_left = steps_left + (steps_left > 0 ? -1 : 1);

    if (!steps_left) {
      if (fn) (*fn)(); // callback
      stop();
    }
  }
}
