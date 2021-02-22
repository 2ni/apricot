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
#include "millis.h"

// to avoid complex division
// we calculate the needed timer count for the sleep function beforehand
// and use the internal function _s_sleep()
#define SLEEP_TIME 50
#define SLEEP_PER UINT32_C(SLEEP_TIME)*1024/1000

int main(void) {
  mcu_init();

  TOUCH button(&PB7);
  // TOUCH button(&PA3, 50, 100); // alu strip to check if someone is sitting on a chair

  pins_disable_buffer();

  uint8_t is_pressed;

  while (1) {
    pins_enable_buffer(&pins_led);
    pins_enable_buffer(&PB7);
    is_pressed = button.is_pressed(&pins_led);
    pins_disable_buffer(&PB7);
    if (is_pressed) {
      // DF("pressed: 0x%02x\n", is_pressed & ~touch_is_pressed_bm);
      // set led when long touch was reached
      if (is_pressed & touch_long_bm) {
        pins_set(&pins_led, 1);
      } else if (is_pressed & touch_verylong_bm) {
        pins_set(&pins_led, 0);
      }
    }

    // released, if MSB not set but lower bits set, eg 0x02
    if (is_pressed && (~is_pressed & touch_is_pressed_bm)) {
      pins_set(&pins_led, 0);
      DF("released: 0x%02x\n", is_pressed);
    }

    // do not sleep if pressed, as we need to measure time for short/long touch (which is on hold if sleeping)
    if (!is_pressed) {
      pins_disable_buffer(&pins_led);
      _sleep(SLEEP_PER, 5);
      pins_enable_buffer(&pins_led);
    }
  }
}
