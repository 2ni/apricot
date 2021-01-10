#include <avr/io.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"
#include "touch.h"
#include "sleep.h"
#include "millis.h"


#define SLEEP_TIME 50
// to avoid complex multiplications we calculate the needed timer count beforehand and use the internal function _s_sleep()
#define SLEEP_PER UINT32_C(SLEEP_TIME)*32768/1000

int main(void) {
  mcu_init();
  millis_init(); // needed for TOUCH.is_pressed()

  TOUCH button(&PB7);
  uint8_t is_pressed;

  while (1) {
    is_pressed = button.is_pressed(&pins_led);
    if (is_pressed) {
      // DF("pressed: 0x%02x\n", is_pressed & ~touch_is_pressed_bm);
      // set led when long touch was reached
      if (is_pressed & touch_long_bm) {
        pins_set(&pins_led, 1);
      }
    }

    // released, if MSB not set but lower bits set, eg 0x02
    if (is_pressed && (~is_pressed & touch_is_pressed_bm)) {
      pins_set(&pins_led, 0);
      DF("released: 0x%02x\n", is_pressed);
    }

    // do not sleep if pressed, as we need to measure time (whic is on hold while sleeping)
    if (!is_pressed) {
      _sleep(SLEEP_PER, 0); // 1638 (precalculated) vs 1650 using sleep_ms(50);
    }
  }
}
