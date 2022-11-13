/*
 * short code to test the apricot mini
 * it uses the pit for a Vusb measurment every 0.5s
 * and stays in deep sleep while idling
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "uart.h"
#include "mcu.h"

volatile uint8_t work = 0;

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;
  work = 1;
}

int main(void) {
  mcu_init(0, 0); // we use our own RTC/PIT

  // disable input ports for low power
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
  PORTB.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
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

  SLPCTRL.CTRLA = SLPCTRL_SMODE_PDOWN_gc | SLPCTRL_SEN_bm; // SLPCTRL_SMODE_PDOWN_gc, SLPCTRL_SMODE_STDBY_gc

  RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm | RTC_RUNSTDBY_bm;
  RTC.PITINTCTRL = RTC_PI_bm; // enable PIT
  RTC.PITCTRLA = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm; // 16384/32768 = 0.5s
  while (RTC.STATUS > 0);
  sei();

  PORTB.DIRSET = PIN5_bm; // LED
  PORTB.DIRSET = PIN6_bm; // enable VCC input
  PORTB.DIRCLR = PIN4_bm; // VCC input, AIN9 on ADC0

  VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
  ADC0.CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp);
  ADC0.MUXPOS = ADC_MUXPOS_AIN9_gc;
  ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_bm; // 8bit

  while (1) {
    if (work) {
      work = 0;
      PORTB.OUTTGL = PIN5_bm;
      PORTB.OUTSET = PIN6_bm; // enable vcc
      ADC0.COMMAND = 1;
      while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
      PORTB.OUTCLR = PIN6_bm; // disable vcc
      // vcc = <adc>*<vref>*100/(adcmax*r1/(r1+r2))
      //     = adc*2.5*10/(255*47/147)
      //     = adc*30.54
      //     5v -> 163.7
      uint16_t vcc = ADC0.RES*250/255*147/47;
      DF("vcc: %u\n", vcc)
      while (uart_is_busy());
      sleep_cpu();
    }
  }
}
