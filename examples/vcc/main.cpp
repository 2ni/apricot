#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"
#include "sleep.h"

// PB4 = ADC0 AIN9
// PA3 = ADC0 AIN3
// PB6 = ADC1 AIN5
// PC5 = ADC1 AIN11
int main(void) {
  mcu_init();

  while (1) {
    /*
    PORTB.DIRCLR = PIN4_bm;

    VREF.CTRLA = VREF_ADC1REFSEL_1V1_gc; // ADC0 = CTRLA, ADC1 = CTRLC
    ADC0.CTRLC = ADC_PRESC_DIV128_gc | ADC_REFSEL_INTREF_gc | (0<<ADC_SAMPCAP_bp);
    ADC0.MUXPOS = ADC_MUXPOS_AIN9_gc;
    ADC0.CTRLA =(1<<ADC_ENABLE_bp) | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc;
    ADC0.COMMAND =1;
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
    ADC0.CTRLA = 0;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    DF("adc: %u\n", ADC0.RES);
    */

    DF("vcc: %u\n", get_vin());
    _delay_ms(2000);
  }
}
