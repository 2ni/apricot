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
#include "sleep.h"

#define SIMPLE

void released(TOUCH::Press_type type, uint32_t ticks) {
  DF("press: %s for %lus\n", type == TOUCH::SHORT ? "SHORT" : (type == TOUCH::LONG ? "LONG" : "VERYLONG"), ticks*8/32768);
}

int main(void) {
  mcu_init();

  TOUCH button(&PB7);
  // TOUCH button(&PA3, 50, 100); // alu strip to check if someone is sitting on a chair

  pins_disable_buffer();

#ifdef SIMPLE

  while (1) {
    button.is_pressed(&released);
    sleep.sleep_for(205);
  }

#elif

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
        start_tick = sleep.current_tick;
      }
    }

    // release
    if (occupied && !is_pressed) {
      occupied = 0;
        pins_set(&pins_led, 0);
      DF("duration: %lus\n", (sleep.current_tick-start_tick)*8/32768);
    }

    sleep.sleep_for(205);
  }

#endif
}
