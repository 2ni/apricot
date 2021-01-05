#include "pins.h"
#include "sleep.h"

pins_t PB0 = { .port = &PORTB, .pin = 0 };
pins_t PB1 = { .port = &PORTB, .pin = 1 };
pins_t PB4 = { .port = &PORTB, .pin = 4, .port_adc = &ADC0, .pin_adc = 9 }; // ADC0/AIN9
pins_t PB5 = { .port = &PORTB, .pin = 5 };
pins_t PB6 = { .port = &PORTB, .pin = 6 };
pins_t PB7 = { .port = &PORTB, .pin = 7 };
pins_t pins_scl = PB0;
pins_t pins_sda = PB1;
pins_t pins_vin = PB4;
pins_t pins_led = PB5;

/*
 * disable digital input buffer on all pins
 * to save power
 */
void pins_disable_buffer() {
  for (uint8_t pin = 0; pin < 8; pin++) {
    (&PORTA.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc;
    (&PORTB.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc;
    (&PORTC.PIN0CTRL)[pin] = PORT_ISC_INPUT_DISABLE_gc;
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
    sleep_ms(20);
    pins_set(pin, 0);
    if (c != num-1) sleep_ms(100);
  }
}

/*
 * get adc value on given pin
 * 10bit resolution
 */
uint16_t pins_getadc(pins_t *pin) {
  pins_output(pin, 0);
  (*pin).port_adc->MUXPOS = ((ADC_MUXPOS_AIN0_gc + (*pin).pin_adc) << 0);

  (*pin).port_adc->CTRLA = (1<<ADC_ENABLE_bp) | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc;
  (*pin).port_adc->COMMAND |= 1;
  while (pins_adc_isrunning(pin));

  (*pin).port_adc->CTRLA = 0;

  return (*pin).port_adc->RES;
}

uint8_t pins_adc_isrunning(pins_t *pin) {
  return !((*pin).port_adc->INTFLAGS & ADC_RESRDY_bm);
}
