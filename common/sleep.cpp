#include "sleep.h"
#include "uart.h"

/*
 * RTC.PER: 16bit max
 * using 32768Hz external crystal
 *
 * max sleep = 2^16*1000*prescaler/32768
 * min sleep = 1*1000*prescaler/32768
 * count = <delay in ms>*32768/1000/prescaler
 *
 * -> 0.03ms  - 2s       (prescaler 1)     00:00:00:02
 *    0.06ms  - 4s       (prescaler 2)     00:00:00:04
 *    0.12ms  - 8s       (prescaler 4)     00:00:00:08
 *    0.24ms  - 16s      (prescaler 8)     00:00:00:16
 *    0.49ms  - 32s      (prescaler 16)    00:00:00:32
 *    0.98ms  - 64s      (prescaler 32)    00:00:01:04
 *    1.95ms  - 128s     (prescaler 64)    00:00:02:08
 *    3.91ms  - 256s     (prescaler 128)   00:00:04:16
 *    7.81ms  - 512s     (prescaler 256)   00:00:08:32
 *    15.625ms- 1024s    (prescaler 512)   00:00:17:04
 * -> 31.25ms - 2048s    (prescaler 1024)  00:00:34:08
 *    62.5ms  - 4096s    (prescaler 2048)  00:01:08:16
 *    125ms   - 8192s    (prescaler 4096)  00:02:16:32
 *    250ms   - 16384s   (prescaler 8192)  00:04:33:04
 *    500ms   - 32768s   (prescaler 16384) 00:09:06:08
 *    1s      - 65536s   (prescaler 32768) 00:18:12:16
 */

ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm;
}

/*
 * prescaler
 * 0 -> :1
 * 1 -> :2
 * 2 -> :4
 * 3 -> :8
 * ...
 * 9 -> :512
 */
void _sleep(uint16_t per, uint8_t prescaler) {
  while (RTC.STATUS > 0);             // wait for all register to be synchronized

  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;     // external 32.768kHz crystal
  RTC.PER = per;
  RTC.INTCTRL = (1 << RTC_OVF_bp);      // overflow interrupt
  RTC.CTRLA = (prescaler<<3) | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;

  RTC.CNT = 0;

  sei();

  SLPCTRL.CTRLA = (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm); // idle, standby or power down
  asm("sleep");

  // while (RTC.STATUS > 0); // somehow this blocks ~1ms and things still work if commented out
  // seems that resetting CLKSEL and CTRLA make the inital RTC.STATUS faster
  RTC.CLKSEL = 0;
  RTC.CTRLA = 0; // turn off RTC
}

/*
 * prescaler 1
 *
 * 1ms - 64'000ms
 * (precision: 1000/1024 = 0.9765625 per bit)
 * max 2^16bit
 *
 * division is slow, avoid it! if you need speed, use _sleep() and precalculate per value
 *
 */
void sleep_ms(uint16_t ms) {
  _sleep((uint32_t)ms*33, 0); // 32768/1000 = 33 -> 1.007080078ms
}

/* prescaler 1024
 *
 * 1s - 65536s (precision: 1000/1024*1024 = 1s per bit)
 * max 2^16bit
 */
void sleep_s(uint16_t seconds) {
  _sleep(seconds, 0xf); // prescaler 2^15 = 32768
}
