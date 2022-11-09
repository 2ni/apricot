/*
 * inspired by
 *   http://coole-basteleien.de/naeherungssensor
 *   https://www.st.com/resource/en/application_note/an4636-demonstration-of-lc-sensor-for-gas-or-water-metering-based-on-stm32l073zeval-and-stm32l476rgnucleo-boards-stmicroelectronics.pdf
 *   https://hum60hz.wordpress.com/2013/12/11/a-diy-crude-inductive-proximity-switch/
 *
 * inductive sensor on PA6 (AINP0, AC2)
 * set AINN0 to Vref (1.5v)
 *     AINP0 -> PA6
 *
 * fromt tests wen can measure the following maximums: 200kHz, min 4mV diff from 1.5v
 *
 * give a short neg and pos pulse
 * set high impedance (input)
 * interrupt generation on rising edge
 * set hysteresis (0, 10mV, 25mV or 50mV)
 *
 *  +3.3v ---+
 *           |  +-C1-+
 *          R1  |    |
 *           |--+-L1-+-R3--> PA6
 *           |  |
 *          R2 C2
 *           |  |
 *  GND -----+--+
 *
 *  R1=100k, R2=83k, L1=15u, C1=50n, C2=1u, R3=50
 *  AC2.AINN=VREF (1.5v)
 *  limiting pin current to 20mA: R3 = (3.3-1.5)/.02 = 90
 *  excitation pulse: 1-5us
 *
 *  instead of R1/R2 use DAC0 or DAC1
 *  instead of AC2.AINN=VREF use DAC2 to set Vref
 *
 *
 * partly based on http://coole-basteleien.de/naeherungssensor
 * we use
 */
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "mcu.h"

#define _wait() do { __asm__ __volatile__ ("nop\n nop\n nop\n nop\n nop\n nop\n nop\n nop\n"); } while (0) // wait ~1us
#define PIN_SIGNAL PIN6_bm
#define SET_HIGH() (PORTA.OUTSET = PIN_SIGNAL)
#define SET_LOW() (PORTA.OUTCLR = PIN_SIGNAL)
#define SET_INPUT() (PORTA.DIRCLR = PIN_SIGNAL)
#define SET_OUTPUT() (PORTA.DIRSET = PIN_SIGNAL)

volatile uint8_t oscillations = 0;
uint8_t oscillations_tmp = 0;
volatile uint8_t c = 0;
volatile uint8_t measure_in_progress = 0;
volatile uint16_t ts_last_osc = 0;

ISR(AC2_AC_vect) {
  AC2.STATUS = AC_CMP_bm; // clear int flag
  oscillations++;
  ts_last_osc = clock.current_tick;
}

/*
 * DIV64 -> measure 64/10MHz=6.4ms
 * timer every 1s: 1000/6.4=156 tics
 */
ISR(TCA0_OVF_vect) {
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  if (++c == 156) {
    c = 0;
    oscillations = 0;
    measure_in_progress = 1;
    // start measure
    PORTA.OUTSET = PIN5_bm;
    SET_OUTPUT();
    SET_LOW();
    _wait();
    _wait();
    _wait();
    _wait();
    _wait();
    _wait();
    SET_HIGH();
    _wait();
    _wait();
    _wait();
    _wait();
    _wait();
    _wait();
    SET_INPUT();
    AC2.CTRLA |= AC_ENABLE_bm;
  }
}

int main(void) {
  mcu_init(0, 0); // disable clock

  // set INTMODE in AC2.CTRLA, AC2.INTCTRL
  // LPMODE off (no low power mode for now)
  // hysteresis: AC_HYSMODE_OFF_gc or AC_HYSMODE_10mV_gc
  // AINP0 = PA6
  VREF.CTRLD = VREF_DAC2REFSEL_1V5_gc; // set DAC2 to 1.5 (also for AC2)
  AC2.CTRLA = AC_INTMODE_BOTHEDGE_gc | AC_LPMODE_DIS_gc | AC_HYSMODE_10mV_gc; // | AC_ENABLE_bm;
  AC2.MUXCTRLA = AC_MUXPOS_PIN0_gc | AC_MUXNEG_VREF_gc;

  // PORTA.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;

  AC2.INTCTRL = AC_CMP_bm;
  AC2.CTRLA |= AC_ENABLE_bm; // enable interrupts

  // oscilloscope debug pin
  PORTA.DIRSET = PIN5_bm;

  // configure timer to trigger measure
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
  TCA0.SINGLE.PER = 1000;
  TCA0.SINGLE.CNT = 0;
  TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV64_gc;

  while (1) {
    if (measure_in_progress) {
      // oscillations should come every ~8us, so we wait 8us and check if value still the same
      oscillations_tmp = oscillations;
      for (uint8_t i=0; i<5; i++) _wait();
      if (oscillations_tmp == oscillations) {
        PORTA.OUTCLR = PIN5_bm;
        measure_in_progress = 0;
        DF("oscillations : %u\n", oscillations);
        // go to sleep, wake up by TCA0 interrupt
      }
    }
  }
}
