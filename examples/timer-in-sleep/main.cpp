#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "clock.h"
#include "touch.h"
#include "pins.h"

#define TEST_SLEEP_CONTINUOUS

#define TICKS_PER_SECOND 1000*32768/1000/(RTC.PER+1)

#define SLEEP_TIME 50
#define SLEEP_PER UINT16_C(SLEEP_TIME)*1024/1000

TOUCH button(&PB7);

int main(void) {
  mcu_init();
  button.init();

  PORTA.DIRSET = PIN7_bm;
  PORTB.DIRSET = PIN5_bm;

  /*
  TOUCH seat(&PA3);
  while (1) {
    DF("seat: %u\n", seat.get_data());
    clock.sleep_for(4096);
  }
  */

  pins_disable_buffer();

  DF("ticks per sec: %u\n", (uint16_t)(TICKS_PER_SECOND));

  uint16_t until = TICKS_PER_SECOND;
  DL("sent");
  // PORTA.OUTSET = PIN7_bm;
  uint16_t now = clock.current_tick;
  _delay_ms(299);
  DL("doing some more stuff");
  clock.sleep_until(now + until);
  // PORTA.OUTCLR = PIN7_bm;
  DL("start listening 1sec from sent");

  // test sleep only once, RTC deactivated afterwards
  DL("sleep once 1s");
  clock.sleep_once(1, clock.SEC);
  DL("done");

#ifdef TEST_SLEEP_CONTINUOUS
  DL("test button every 50ms with RTC on all the time");
  // clock.init(32767);
  clock.init();
  uint8_t is_taken = 0;
  uint8_t is_pressed = 0;
  uint32_t start_tick = 0;
  while (1) {
    pins_enable_buffer(&PB7);
    is_pressed = button.is_pressed();
    pins_disable_buffer(&PB7);
    if (is_pressed && !is_taken) {
      is_taken = 1;
      PORTB.OUTSET = PIN5_bm;
      start_tick = clock.current_tick;
      // DF("start: %lu\n", start_tick);
    }

    if (!is_pressed && is_taken) {
      is_taken = 0;
      PORTB.OUTCLR = PIN5_bm;
      uint32_t end_tick = clock.current_tick;
      // DF("duration: %lus\n", clock.current_tick + (clock.current_tick > start_tick ? -start_tick : start_tick));
      DF("duration (50.0488ms precision, because of sleep): %lums\n", clock.ticks2ms(end_tick + (end_tick > start_tick ? -start_tick : start_tick)));
    }
    // clock.sleep_for(1);
    clock.sleep_for(205); // 8/32768*1000*205 = 50.048828125ms
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
    clock.sleep_once(SLEEP_PER, 5);
    // PORTA.OUTSET = PIN7_bm;
  }
#endif
}
