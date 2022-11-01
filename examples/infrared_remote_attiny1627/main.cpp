#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "uart.h"
#include "clock.h"

CLOCK clock;

// make mcu=attiny1627 common="uart clock" port=3 clk=5000000
// examples: https://github.com/search?q=attiny1627+user:microchiptech+user:microchip-pic-avr-examples&type=Repositories

#define LED_PORT PORTC
#define LED1_PIN PIN4_bm
#define LED2_PIN PIN5_bm
#define LED1_ON()  (LED_PORT.OUTSET = LED1_PIN)
#define LED1_OFF() (LED_PORT.OUTCLR = LED1_PIN)
#define LED1_TGL() (LED_PORT.OUTTGL = LED1_PIN)
#define LED2_ON()  (LED_PORT.OUTSET = LED2_PIN)
#define LED2_OFF() (LED_PORT.OUTCLR = LED2_PIN)
#define LED2_TGL() (LED_PORT.OUTTGL = LED2_PIN)

#define IRREC_PORT PORTA
#define IRREC_PIN PIN6_bm
#define IR_REC_ON() (IRREC_PORT.OUTSET = IRREC_PIN)
#define IR_REC_OFF() (IRREC_PORT.OUTCLR = IRREC_PIN)

#define SHIELD_PORT PORTB
#define SHIELD_PIN PIN7_bm
#define SHIELD_ON()  (SHIELD_PORT.OUTSET = SHIELD_PIN)
#define SHIELD_OFF() (SHIELD_PORT.OUTCLR = SHIELD_PIN)

/*
 * touch = 0..7
 */
uint16_t read_touch(uint8_t touch) {
  // J1 (T0): PC3/AIN15
  // J2 (T1): PC2/AIN14
  // J3 (T2): PC1/AIN13
  // J4 (T3): PC0/AIN12
  // J5 (T4): PB5/AIN8
  // J6 (T5): PB4/AIN9
  // J7 (T6): PB1/AIN10
  // J8 (T7): PB0/AIN11

  if (touch > 8) return 0;

  uint8_t pin;
  uint8_t adc;
  PORT_t *port;

  if (touch < 4) {
    pin = 1 << (3 - touch);
    adc = 15 - touch;
    port = &PORTC;
  } else if (touch < 6) {
    pin = 1 << (9 - touch);
    adc = 4 + touch;
    port = &PORTB;
  } else {
    pin = 1 << (7 - touch);
    adc = 4 + touch;
    port = &PORTB;
  }

  port->OUTSET = pin;

  // empty s/h capacitor
  ADC0.MUXPOS = ADC_MUXPOS_GND_gc;
  __builtin_avr_delay_cycles(10);

  // charge Ctouch by setting output
  SHIELD_ON();
  port->DIRSET = pin;
  __builtin_avr_delay_cycles(10);

  port->DIRCLR = pin;
  SHIELD_OFF();

  // transfer charge to Csh
  ADC0.MUXPOS = adc;

  // read s/h voltage
  ADC0.COMMAND |= ADC_START_IMMEDIATE_gc;
  while (!(ADC0.INTFLAGS & ADC_SAMPRDY_bm));
  return ADC0.SAMPLE;
}

/*
 * binary representation of uint8_t
 * char buf[9];
 * bitwise(buf, some_variable);
 * buf[8] = '\0';
 * DF("value: %s\n", buf);
 */
void bitwise(char *buf, uint8_t value) {
  for (uint8_t i=0; i<8; i++) {
    buf[i] = value & 0x80 ? '1' : '0';
    value <<= 1;
  }
}

void sleep() {
  PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;

  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();
  sleep_cpu();
}

int main(void) {
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_4X_gc | CLKCTRL_PEN_bm); // 5MHz

  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_RUNSTDBY_bm);
  while (CLKCTRL.MCLKSTATUS & CLKCTRL_XOSC32KS_bm);

  /*
  // test sleep power consumption -> 0.85uA
  LED_PORT.DIRSET = LED1_PIN;
  for (uint8_t i=0; i<3; i++) {
    LED_PORT.OUTSET = LED1_PIN;
    _delay_ms(20);
    LED_PORT.OUTCLR = LED1_PIN;
    _delay_ms(100);
  }
  sleep();
  while(1);
  */

  SLPCTRL.CTRLA = (SLPCTRL_SMODE_STDBY_gc | SLPCTRL_SEN_bm);

  clock.init();

  PORTMUX.USARTROUTEA = PORTMUX_USART0_ALT1_gc;
  LED_PORT.DIRSET = LED1_PIN;            // PC4 (yellow led)
  LED_PORT.DIRSET = LED2_PIN;            // PC5 (blue led)
  SHIELD_PORT.DIRSET = SHIELD_PIN;       // PB7 (shield)
  IRREC_PORT.DIRSET = IRREC_PIN;         // PA6 (enable IRrec module)

  for (uint8_t i=0; i<3; i++) {
    LED1_ON();
    _delay_ms(20);
    LED1_OFF();
    if (i < 3) _delay_ms(100);
  }
  uart_init();

  uint16_t defaults[8] = {0};
  uint8_t touches = 0x00;
  uint8_t touches_last = 0x00;

  ADC0.CTRLA = ADC_ENABLE_bm;
  ADC0.CTRLB = ADC_PRESC_DIV4_gc;
  ADC0.CTRLC = (5<<ADC_TIMEBASE_gp) | ADC_REFSEL_VDD_gc; // ceil(timebase F_CPU*0.000001), according to p.409
  // ADC0.CTRLE = 17; // adc clock cycles sampling time
  ADC0.COMMAND = ADC_MODE_SINGLE_12BIT_gc; // no diff mode, only ADC0.MUXPOS is used, ADC_START_IMMEDIATE_gc to start

  // disable input buffer for better adc reading
  PORTC.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN3CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN0CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN1CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN4CTRL |= PORT_ISC_INPUT_DISABLE_gc;
  PORTB.PIN5CTRL |= PORT_ISC_INPUT_DISABLE_gc;

  DL("touch default values");

  read_touch(7);

  for (uint8_t i=0; i<8; i++) {
    defaults[i] = 0;
    for (uint8_t ii=0; ii<8; ii++) {
      defaults[i] += read_touch(i);
    }
    defaults[i] >>= 3;
    DF("J%u: %u\n", i+1, defaults[i]);
  }
  DL("******");
  while (uart_is_busy());

  // _delay_ms(1000);

  while (1) {
    for (uint8_t i=0; i<8; i++) {
      uint16_t v = read_touch(i);
      // DF("J%i: %u\n", i+1, v);
      if (v > (defaults[i] + 500)) {
        touches |= (1<<i);
      } else {
        touches &= ~(1<<i);
      }
    }
    // DF("0x%02x\n", touches);

    if (touches != touches_last) {
      touches_last = touches;
      if (touches) {
        LED1_ON();
        uart_tuple("touches", touches, 2);
        while (uart_is_busy());
      } else {
        LED1_OFF();
      }
    }

    clock.sleep_for(410); // 100ms
  }
}
