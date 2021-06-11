#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"
#include "stepper.h"

// https://www.opendcc.de/elektronik/opendecoder/opendecoder_sw_schalt_e.html
// make mcu=attiny1604 flash

// DCC PA5
// LIMIT1 PB1
// LIMIT2 PB0

STEPPER stepper;

int8_t direction = 0;
int8_t warn = 0;

ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  switch (in) {
    case 'r':
      direction = 1;
      break;
    case 'l':
      direction = -1;
      break;
    case '\n':
      pins_flash(&pins_led, 1, 100);
      break;
  }
}

ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;
  PORTA.INTFLAGS = flags; // clear flags

  // DCC input
  if (flags & PORT_INT3_bm) {
    warn = 1;
  }

  // stepper limits
  if (flags & (PORT_INT1_bm | PORT_INT2_bm)) {
    stepper.stop();
  }
}

/*
 * + -> right move
 * - -> left move
 */
int main(void) {
  mcu_init(1);
  clock.init(0); // 1 tick = 1/32768 = 30.6us

  pins_t INA1 = PA7;
  pins_t INA2 = PA6;
  pins_t INB1 = PB2;
  pins_t INB2 = PB3;
  pins_t LIMIT1 = PB1;
  pins_t LIMIT2 = PB0;
  pins_t DCC = PA5;

  pins_output(&DCC, 0); // set as input

  pins_pullup(&LIMIT1, 1);
  pins_pullup(&LIMIT2, 1);
  PORTB.PIN1CTRL |= PORT_ISC_FALLING_gc; // LIMIT1
  PORTB.PIN0CTRL |= PORT_ISC_FALLING_gc; // LIMIT2
  PORTA.PIN5CTRL |= PORT_ISC_RISING_gc; // DCC

  stepper.init(&INA1, &INA2, &INB1, &INB2);

  while (1) {
    if (warn) {
      DF("clock: %lu\n", clock.current_tick);
      pins_flash(&pins_led, 1, 100);
      warn = 0;
    }

    if (direction == -1 || direction == 1) {
      int16_t m = 100 * direction;
      DF("move: %i\n", m);
      // pins_set(&pins_led, 1);
      stepper.move(m, 0);
      // pins_set(&pins_led, 0);
      direction = 0;
    }
  }
}
