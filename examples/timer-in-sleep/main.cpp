#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "sleepv2.h"
#include "touch.h"
#include "pins.h"

#define TEST_SLEEP_CONTINUOUS

#define TICKS_PER_SECOND 1000*32768/1000/(RTC.PER+1)

#define SLEEP_TIME 50
#define SLEEP_PER UINT16_C(SLEEP_TIME)*1024/1000

SLEEP sleep;

int main(void) {
  mcu_init();
  sleep.init();
  PORTA.DIRSET = PIN7_bm;
  PORTB.DIRSET = PIN5_bm;
  TOUCH button(&PB7);

  DF("ticks per sec: %u\n", (uint16_t)(TICKS_PER_SECOND));

  uint16_t until = TICKS_PER_SECOND;
  DL("sent");
  // PORTA.OUTSET = PIN7_bm;
  uint16_t now = sleep.current_tick;
  _delay_ms(299);
  DL("doing some more stuff");
  sleep.sleep_until(now + until);
  // PORTA.OUTCLR = PIN7_bm;
  DL("start listening");

  // test sleep only once, RTC deactivated afterwards
  DL("sleep once 1s");
  sleep.sleep_once(1, sleep.SEC);
  DL("done");

#ifdef TEST_SLEEP_CONTINUOUS
  DL("test button every 50ms with RTC on all the time");
  sleep.init();
  while (1) {
    if (button.is_pressed()) {
      PORTB.OUTSET = PIN5_bm;
      DF("current tick: %lu\n", sleep.current_tick);
    } else {
      PORTB.OUTCLR = PIN5_bm;
    }
    PORTA.OUTCLR = PIN7_bm;
    sleep.sleep_for(205); // 8/32768*1000*205 = 50.048828125ms
    PORTA.OUTSET = PIN7_bm;
  }
#else
  // test sleep modes which make use of sleep once events
  // this uses less power, as it wakes up only every 50ms
  DL("wake up every 50ms with single sleep");
  while (1) {
    if (button.is_pressed()) {
      PORTB.OUTSET = PIN5_bm;
    } else {
      PORTB.OUTCLR = PIN5_bm;
    }

    // PORTA.OUTCLR = PIN7_bm;
    sleep.sleep_once(SLEEP_PER, 5);
    // PORTA.OUTSET = PIN7_bm;
  }
#endif
}
