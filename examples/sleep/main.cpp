#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include "uart.h"
#include "sleep.h"
#include "pins.h"
#include "mcu.h"
#include "lorawan.h"
#include "ssd1306.h"

LORAWAN lora;

int main(void) {
  mcu_init();
  lora.rfm95.init(); // ensure potential rfm95 is set to sleep

  pins_disable_buffer();

  /*
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  sleep_cpu();
  */

  PORTB.DIRSET = PIN5_bm; // set PB5 (led) as output
  uint8_t led_is_on = 0;
  while (1) {
    led_is_on = PORTB.IN & PIN5_bm;
    DF("blink: 0x%02x\n", led_is_on);
    PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
    sleep_s(led_is_on ? 1 : 5);
    PORTB.PIN5CTRL = 0;
    PORTB.OUTTGL = PIN5_bm;
  }
}
