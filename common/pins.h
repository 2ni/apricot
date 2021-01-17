#ifndef __PINS_H__
#define __PINS_H__

#include <avr/io.h>

typedef struct {
  PORT_t  *port;
  uint8_t pin;
  ADC_t   *port_adc;
  uint8_t pin_adc;
} pins_t;

extern pins_t PA1, PA2, PA3, PA4, PA5, PA6, PA7;
extern pins_t PB0, PB1, PB4, PB5, PB6, PB7, pins_scl, pins_sda, pins_vin, pins_led;
extern pins_t PC0, PC1, PC2, PC3, PC4, PC5, pins_sck, pins_miso, pins_mosi;
extern pins_t pins_csrfm, pins_dio0, pins_dio1;

void     pins_disable_buffer();
void     pins_output(pins_t *pin, uint8_t value);
void     pins_set(pins_t *pin, uint8_t value);
void     pins_toggle(pins_t *pin);
uint8_t  pins_get(pins_t *pin);
void     pins_pullup(pins_t *pin, uint8_t value);
void     pins_flash(pins_t *pin, uint8_t num=3, uint8_t duration=100);
uint16_t pins_getadc(pins_t *pin);
uint8_t  pins_adc_isrunning(pins_t *pin);

#endif
