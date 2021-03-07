#include <avr/interrupt.h>
#include "clock.h"
#include "uart.h"

CLOCK* CLOCK::clock_ptr;

ISR(RTC_CNT_vect) {
  // PORTA.OUTSET = PIN7_bm;
  // PORTA.OUTCLR = PIN7_bm;
  RTC.INTFLAGS = RTC_OVF_bm;
  CLOCK::clock_ptr->current_tick++;
}

CLOCK::CLOCK() {
  clock_ptr = this;
  _is_running = 0;
  _mode = CONTINUOUS;
  RTC.PER = 7; // set default so we can use ms2ticks beforehand
}

/*
 * RTC.PER configuration
 *
 * ticks:  <ms>*32768/1000/(RTC.PER+1);
 * ms   : <ticks>*(RTC.PER+1)/32768*1000
 *
 * 32767 -> 1024:  32768/32768 = 1Hz      = 1s                            -> 1uA
 * 31    -> 32   : 32768/32    = 1024Hz   = 0.9765625ms (1024 ticks/sec)  -> 6uA
 * 33    -> 34   : 32768/34    = 963.8Hz  = 1.037ms                       -> 5.9uA
 * 17    -> 18   : 32768/18    = 1820.4Hz = 0.549316msa                   -> 10uA
 * 7     ->  8   : 32768/8     = 4096Hz   = 244.140625us (4096 ticks/sec) -> 22uA (high: 1.7us, low: 242us)
 *
 * max duration for current_tick: 2^16 * (RTC.PER+1)/32768
 *     uint16_t  uint32_t
 * 7 : 16sec     12d 03h 16m 16s
 * 31: 64sec     48d 13h 05m 04s
 *
 */
void CLOCK::init(uint16_t per) {
  current_tick = 0;

  stop();
  while (RTC.STATUS >0);

  RTC.PER = per;
  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc; // external 32.768kHz
  RTC.INTCTRL =  RTC_OVF_bm;
  RTC.CNT = 0;
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
  _is_running = 1;
  sei();
}

uint8_t CLOCK::is_continuous() {
  return _mode == CONTINUOUS && _is_running;
}

void CLOCK::start() {
  _is_running = 1;
  RTC.CNT = 0;
  RTC.CTRLA |= RTC_RTCEN_bm;
}

void CLOCK::stop() {
  // RTC.CLKSEL = 0;
  _is_running = 0;
  RTC.CTRLA &= ~RTC_RTCEN_bm;
}

void CLOCK::sleep_until(uint32_t tick_until) {
  _mode = CONTINUOUS;
  while (current_tick != tick_until) {
    // PORTA.OUTCLR = PIN7_bm;
    __asm__ __volatile__ ( "sleep" "\n\t" :: );
    // PORTA.OUTSET = PIN7_bm;
  }
}

void CLOCK::sleep_for(uint32_t ticks) {
  sleep_until(current_tick + ticks);
}

/*
 * to spare some time it can be precalculated if fixed, eg
 * CLOCK clock
 * #define TICKS_SLEEP <ms>*32768/1000/(RTC.PER+1)
 */
uint32_t CLOCK::ms2ticks(uint32_t ms) {
  return ms * 32768/1000/(RTC.PER+1);
}

uint32_t CLOCK::ticks2ms(uint32_t ticks) {
  return (ticks * (RTC.PER+1)*1000)/32768;
}

/*
 * ms       : per*1000*2^prescaler/32768
 * precision: 1000*2^prescaler/32768
 * max      : 2^16*precision
 */
void CLOCK::sleep_once(uint16_t per, uint16_t prescaler) {
  _mode = SINGLE;
  // PORTA.OUTSET = PIN7_bm;
  _is_running = 0;
  stop();
  while (RTC.STATUS > 0);
  // PORTA.OUTCLR = PIN7_bm;
  RTC.PER = per;
  RTC.CTRLA = (prescaler<<3) | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
  RTC.CNT = 0;
  __asm__ __volatile__ ( "sleep" "\n\t" :: );
  stop();
}

/*
 * if you don't care about slow divisions for msec
 * you can also use this simpler function
 * else use sleep_once(per, prescaler)
 * MSEC: 0.98ms - 64s
 * SEC : 1s     - 65535s
 */
void CLOCK::sleep_once(uint16_t duration, Duration_type ms_or_s) {
  if (ms_or_s == MSEC) {
    sleep_once(((uint32_t)duration*1024)/1000, 5);
  } else if (ms_or_s == SEC) {
    sleep_once(duration, 15);
  }
}
