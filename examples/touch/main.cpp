/*
 * example of capacitive touch with optimized
 * sleep mode
 * every 50ms the touch is checking for pressing
 * the measurement lasts ~250us
 */
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"
#include "touch.h"
#include "clock.h"

// 1: loop over 1sec
// 2: use simple is_pressed looping over 50ms
// 3: extended example as 2 but done manually
#define EXAMPLE 2

TOUCH button(&PB7);

void released(TOUCH::Press_type type, uint32_t ticks) {
  DF("press: %s for %lus\n", type == TOUCH::SHORT ? "SHORT" : (type == TOUCH::LONG ? "LONG" : "VERYLONG"), ticks*8/32768);
}

void released_seconds(TOUCH::Press_type type, uint32_t ticks) {
  DF("press: %s for %lus\n", type == TOUCH::SHORT ? "SHORT" : (type == TOUCH::LONG ? "LONG" : "VERYLONG"), ticks);
}

int main(void) {
  mcu_init();
  button.init();
  // button.init(10, 30); // alu strip to check if someone is sitting on a chair

  pins_disable_buffer();

#if EXAMPLE == 1

  // set to 1sec intervalls
  clock.init(32767);
  while (1) {
    button.is_pressed(&released_seconds, 3, 5);
    clock.sleep_for(1);
  }

#elif EXAMPLE == 2
  while (1) {
    button.is_pressed(&released);
    clock.sleep_for(205); // 50ms*32768/8/1000 = 204.8
  }

#elif EXAMPLE == 3

  uint32_t start_tick = 0;
  uint8_t occupied = 0;
  uint8_t is_pressed = 0;
  while (1) {
    is_pressed = button.is_pressed();
    if (is_pressed) {
      // inital press
      if (!occupied) {
        occupied = 1;
        pins_set(&pins_led, 1);
        start_tick = clock.current_tick;
      }
    }

    // release
    if (occupied && !is_pressed) {
      occupied = 0;
        pins_set(&pins_led, 0);
      DF("duration: %lus\n", (clock.current_tick-start_tick)*8/32768);
    }

    clock.sleep_for(205);
  }

#endif
}
