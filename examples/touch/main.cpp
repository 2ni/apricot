#include <util/delay.h>
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
  millis_init(); // needed for TOUCH.pressed

  TOUCH button(&PB7);

  while (1) {
    TOUCH::STATUS status = button.pressed(2000, &pins_led);
    if (status == TOUCH::LONG) {
      DL("long press");
    } else if (status == TOUCH::SHORT) {
      DL("short press");
    }
    _sleep(SLEEP_PER, 0); // 1638 (precalculated) vs 1650 using sleep_ms(50);
  }
}
