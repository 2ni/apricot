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

int16_t direction = 0;
volatile uint8_t warn = 0;
volatile uint8_t limit_reached = 0;
uint8_t speed = 5;

ISR(USART0_RXC_vect) {
  uint8_t in = USART0.RXDATAL;
  switch (in) {
    case 'y':
      direction = -100;
      break;
    case '.':
      direction = 100;
      break;
    case '<':
      direction = -1000;
      break;
    case '-':
      direction = 1000;
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
  if (flags & PORT_INT5_bm) {
    warn = 1;
  }
}

ISR(PORTB_PORT_vect) {
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; // clear flags

  // stepper limits
  if (flags & (PORT_INT1_bm | PORT_INT0_bm)) {
    limit_reached = 1;
  }
}

void stopped() {
  pins_set(&pins_led, 0);
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

  // set to left position if unknown at start
  if (pins_get(&LIMIT1) && pins_get(&LIMIT2)) {
    DL("move to home");
    stepper.move(-1000, speed);
    while (pins_get(&LIMIT1)) {
      stepper.loop();
    }
  }

  while (1) {
    if (warn) {
      DF("clock: %lu\n", clock.current_tick);
      pins_flash(&pins_led, 1, 100);
      warn = 0;
    }

    if (limit_reached) {
      stepper.stop();
      pins_set(&pins_led, 0);
      limit_reached = 0;
    }

    if (direction != 0) {
      // stop on limits
      if (((!pins_get(&LIMIT1) && direction < 0) || (!pins_get(&LIMIT2) && direction > 0))) {
        continue;
      }

      DF("move: %i\n", direction);
      pins_set(&pins_led, 1);
      stepper.move(direction, speed); // 30 ticks = 1ms
      direction = 0;
    }

    stepper.loop(&stopped);
  }
}
