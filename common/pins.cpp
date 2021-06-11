#include <util/delay.h>
#include "pins.h"
pins_t PA1 = { .port = &PORTA, .pin = 1, .port_adc = &ADC0, .pin_adc = 1 };  // TXD (reserved for uart)
pins_t PA2 = { .port = &PORTA, .pin = 2, .port_adc = &ADC0, .pin_adc = 2 };  // RXD (reserved for uart)
pins_t PA3 = { .port = &PORTA, .pin = 3, .port_adc = &ADC0, .pin_adc = 3 };
pins_t PA4 = { .port = &PORTA, .pin = 4, .port_adc = &ADC0, .pin_adc = 4 };
pins_t PA5 = { .port = &PORTA, .pin = 5, .port_adc = &ADC0, .pin_adc = 5 };
pins_t PA6 = { .port = &PORTA, .pin = 6, .port_adc = &ADC0, .pin_adc = 6 };
pins_t PA7 = { .port = &PORTA, .pin = 7, .port_adc = &ADC0, .pin_adc = 7 };

pins_t PB0 = { .port = &PORTB, .pin = 0, .port_adc = &ADC0, .pin_adc = 11 }; // SCL
pins_t PB1 = { .port = &PORTB, .pin = 1, .port_adc = &ADC0, .pin_adc = 10 }; // SDA
pins_t pins_scl = PB0;
pins_t pins_sda = PB1;

#ifdef __AVR_ATtiny3217__
pins_t PB4 = { .port = &PORTB, .pin = 4, .port_adc = &ADC0, .pin_adc = 9 };  // VIN
pins_t PB5 = { .port = &PORTB, .pin = 5, .port_adc = &ADC0, .pin_adc = 8 };  // LED
pins_t PB6 = { .port = &PORTB, .pin = 6, .port_adc = &ADC1, .pin_adc = 5 };
pins_t PB7 = { .port = &PORTB, .pin = 7, .port_adc = &ADC1, .pin_adc = 4 };
pins_t pins_vin = PB4;
pins_t pins_led = PB5;
#elif defined(__AVR_ATtiny1604__)
pins_t PB2 = { .port = &PORTB, .pin = 2 };
pins_t PB3 = { .port = &PORTB, .pin = 3 };

pins_t pins_led = PA4;
#endif

#ifdef __AVR_ATtiny3217__
pins_t PC0 = { .port = &PORTC, .pin = 0, .port_adc = &ADC1, .pin_adc = 6 };  // SCK
pins_t PC1 = { .port = &PORTC, .pin = 1, .port_adc = &ADC1, .pin_adc = 7 };  // MISO
pins_t PC2 = { .port = &PORTC, .pin = 2, .port_adc = &ADC1, .pin_adc = 8 };  // MOSI
pins_t PC3 = { .port = &PORTC, .pin = 3, .port_adc = &ADC1, .pin_adc = 9 };  // CS_RFM
pins_t PC4 = { .port = &PORTC, .pin = 4, .port_adc = &ADC1, .pin_adc = 10 }; // DIO0
pins_t PC5 = { .port = &PORTC, .pin = 5, .port_adc = &ADC1, .pin_adc = 11 }; // DIO1
pins_t pins_sck = PC0;
pins_t pins_miso = PC1;
pins_t pins_mosi = PC2;
pins_t pins_csrfm = PC3;
pins_t pins_dio0 = PC4;
pins_t pins_dio1 = PC5;
#endif

/*
 * disable digital input buffer on all pins
 * to save power
 * single
 */
void pins_disable_buffer(pins_t *pin) {
  if (pin == 0) {
    for (uint8_t pin = 0; pin < 8; pin++) {
      (&PORTA.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc;
      (&PORTB.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc;
      (&PORTC.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc;
    }
  } else {
    register8_t *pctrl = &(*pin).port->PIN0CTRL;
    pctrl += (*pin).pin;
    *pctrl = PORT_ISC_INPUT_DISABLE_gc;
  }
}

void pins_enable_buffer(pins_t *pin) {
  if (pin == 0) {
    for (uint8_t pin = 0; pin < 8; pin++) {
      (&PORTA.PIN0CTRL)[pin] = PORT_ISC_INTDISABLE_gc;
      (&PORTB.PIN0CTRL)[pin] = PORT_ISC_INTDISABLE_gc;
      (&PORTC.PIN0CTRL)[pin] = PORT_ISC_INTDISABLE_gc;
    }
  } else {
    register8_t *pctrl = &(*pin).port->PIN0CTRL;
    pctrl += (*pin).pin;
    *pctrl = PORT_ISC_INTDISABLE_gc;
  }
}

/*
 * set direction (input, output)
 * value = 1 -> output
 * value = 0 -> input
 */
void pins_output(pins_t *pin, uint8_t value) {
  if (value) (*pin).port->DIRSET = (1<<(*pin).pin);
  else (*pin).port->DIRCLR = (1<<(*pin).pin);
}

/*
 * set or clear pin value
 * value = 1 -> 3.3v
 * value = 0 -> 0v
 */
void pins_set(pins_t *pin, uint8_t value) {
  if (value) (*pin).port->OUTSET = (1<<(*pin).pin);
  else (*pin).port->OUTCLR = (1<<(*pin).pin);
}

/*
 * toggle pin value
 */
void pins_toggle(pins_t *pin) {
  (*pin).port->OUTTGL = (1<<(*pin).pin);
}

/*
 * return value of a pin
 */
uint8_t pins_get(pins_t *pin) {
  return ((*pin).port->IN & (1<<(*pin).pin)) != 0;
}

void pins_pullup(pins_t *pin, uint8_t value) {
  register8_t *pctrl = &(*pin).port->PIN0CTRL;
  pctrl += (*pin).pin;

  if (value) *pctrl |= PORT_PULLUPEN_bm;
  else *pctrl &= ~PORT_PULLUPEN_bm;
}

void pins_flash(pins_t *pin, uint8_t num, uint8_t duration) {
  pins_output(pin, 1);
  for (uint8_t c=0; c<num; c++) {
    pins_set(pin, 1);
    _delay_ms(20); // TOOD replace with SLEEP pointer to sleep->is_continuous and use appropriate sleep function
    pins_set(pin, 0);
    if (c != num-1) _delay_ms(100);
  }
}

/*
 * get adc value on given pin
 * 10bit resolution
 */
uint16_t pins_getadc(pins_t *pin) {
  pins_output(pin, 0); // set input
  (*pin).port_adc->MUXPOS = ((ADC_MUXPOS_AIN0_gc + (*pin).pin_adc) << 0);

  (*pin).port_adc->CTRLA = (1<<ADC_ENABLE_bp) | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc;
  (*pin).port_adc->COMMAND |= 1;
  while (pins_adc_isrunning(pin));

  (*pin).port_adc->CTRLA = 0;

  (*pin).port_adc->INTFLAGS = ADC_RESRDY_bm; // clear interrupt

  return (*pin).port_adc->RES;
}

uint8_t pins_adc_isrunning(pins_t *pin) {
  return !((*pin).port_adc->INTFLAGS & ADC_RESRDY_bm);
}
