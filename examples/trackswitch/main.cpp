#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "mcu.h"
#include "pins.h"
#include "stepper.h"

STEPPER stepper;

int8_t direction = 0;

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

/*
 * + -> right move
 * - -> left move
 */
int main(void) {
  mcu_init();


  pins_t INA1 = PA5;
  pins_t INA2 = PA4;
  pins_t INB1 = PA7;
  pins_t INB2 = PA6;
  pins_t LIMIT1 = PA1;
  pins_t LIMIT2 = PA2;

  stepper.init(&INA1, &INA2, &INB1, &INB2);

  while (1) {
    if (direction == -1 || direction == 1) {
      int16_t m = 100 * direction;
      DF("move: %i\n", m);
      stepper.move(m, 0);
      direction = 0;
    }
  }

  stepper.move(-100, 0);
  while (1);
  pins_flash(&pins_led);
  _delay_ms(1000);
  stepper.move(100, 0);

  pins_output(&INA1, 1);
  pins_output(&INA2, 1);
  pins_output(&INB1, 1);
  pins_output(&INB2, 1);
  pins_output(&LIMIT1, 0);
  pins_output(&LIMIT2, 0);


  pins_set(&INA2, 1);
  pins_set(&INB1, 1);

  while (1) {
    pins_toggle(&pins_led);
    if (pins_get(&pins_led)) {
      DL("LED on");
      pins_pullup(&LIMIT1, 1);
      pins_pullup(&LIMIT2, 1);
      DF("limit 1: %u, limit 2: %u\n", pins_get(&LIMIT1), pins_get(&LIMIT2));
      pins_pullup(&LIMIT1, 0);
      pins_pullup(&LIMIT2, 0);
    }
    _delay_ms(500);
  }
}
