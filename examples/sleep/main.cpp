#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include "uart.h"
#include "pins.h"
#include "mcu.h"
#include "lorawan.h"
#include "ssd1306.h"

LORAWAN lora;

int main(void) {
  mcu_init();
  lora.rfm95.init(); // ensure potential rfm95 is set to sleep

  pins_disable_buffer();

  // for low power
  // disconnect 3.3u, updi
  // set txd (pa1) as input or disconnect
  /*
  PORTA.DIRCLR = PIN1_bm;
  set_sleep_mode(SLEEP_MODE_STANDBY); // or SLEEP_MODE_STANDBY, SLEEP_MODE_PWR_DOWN
  clock.stop(); // disable interrupts from rtc
  sleep_enable();
  sleep_cpu();
  DL("sleep");
  */

  /*
  SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm; // SLPCTRL_SMODE_PDOWN_gc, SLPCTRL_SMODE_STDBY_gc
  sei();
  sleep_cpu();
  */

  /*
  DL("go to sleep");
  while(uart_is_busy());
  clock.stop();
  __asm__ __volatile__ ( "sleep" "\n\t" :: );
  */

  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output
  uint8_t led_is_on = 0;
  while (1) {
    led_is_on = PORTB.IN & PIN5_bm;
    DF("blink: 0x%02x\n", led_is_on);
    PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
    clock.sleep_for(led_is_on ? 4096 : 20480); // 1sec : 5sec
    PORTB.PIN5CTRL = 0;
    PORTB.OUTTGL = PIN5_bm;
  }
}
