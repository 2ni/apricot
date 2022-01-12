#ifndef __AVR_IO__
#define __AVR_IO__

#include <stdint.h>

typedef volatile uint8_t register8_t;
typedef volatile uint16_t register16_t;
typedef volatile uint32_t register32_t;

/*
typedef signed char       int8_t;
typedef unsigned char     uint8_t;
typedef signed short      int16_t;
typedef unsigned short    uint16_t;
typedef signed int        int32_t;
typedef unsigned int      uint32_t;
*/

#define PIN0_bm 0x01
#define PIN0_bp 0
#define PIN1_bm 0x02
#define PIN1_bp 1
#define PIN2_bm 0x04
#define PIN2_bp 2
#define PIN3_bm 0x08
#define PIN3_bp 3
#define PIN4_bm 0x10
#define PIN4_bp 4
#define PIN5_bm 0x20
#define PIN5_bp 5
#define PIN6_bm 0x40
#define PIN6_bp 6
#define PIN7_bm 0x80
#define PIN7_bp 7

#define PORT_PULLUPEN_bm  0x08  /* Pullup enable bit mask. */


typedef struct PORT_struct
{
  register8_t DIR;  /* Data Direction */
  register8_t DIRSET;  /* Data Direction Set */
  register8_t DIRCLR;  /* Data Direction Clear */
  register8_t DIRTGL;  /* Data Direction Toggle */
  register8_t OUT;  /* Output Value */
  register8_t OUTSET;  /* Output Value Set */
  register8_t OUTCLR;  /* Output Value Clear */
  register8_t OUTTGL;  /* Output Value Toggle */
  register8_t IN;  /* Input Value */
  register8_t INTFLAGS;  /* Interrupt Flags */
  register8_t reserved_0x0A;
  register8_t reserved_0x0B;
  register8_t reserved_0x0C;
  register8_t reserved_0x0D;
  register8_t reserved_0x0E;
  register8_t reserved_0x0F;
  register8_t PIN0CTRL;  /* Pin 0 Control */
  register8_t PIN1CTRL;  /* Pin 1 Control */
  register8_t PIN2CTRL;  /* Pin 2 Control */
  register8_t PIN3CTRL;  /* Pin 3 Control */
  register8_t PIN4CTRL;  /* Pin 4 Control */
  register8_t PIN5CTRL;  /* Pin 5 Control */
  register8_t PIN6CTRL;  /* Pin 6 Control */
  register8_t PIN7CTRL;  /* Pin 7 Control */
  register8_t reserved_0x18;
  register8_t reserved_0x19;
  register8_t reserved_0x1A;
  register8_t reserved_0x1B;
  register8_t reserved_0x1C;
  register8_t reserved_0x1D;
  register8_t reserved_0x1E;
  register8_t reserved_0x1F;
} PORT_t;

extern PORT_t FakePort;
#define PORTA (*(PORT_t *) &FakePort)
#define PORTB (*(PORT_t *) &FakePort)
#define PORTC (*(PORT_t *) &FakePort)

#define _WORDREGISTER(regname)   \
    __extension__ union \
    { \
        register16_t regname; \
        struct \
        { \
            register8_t regname ## L; \
            register8_t regname ## H; \
        }; \
    }

#define ADC_RESRDY_bm  0x01
#define ADC_ENABLE_bp  0  /* ADC Enable bit position. */
#define ADC_FREERUN_bp  1  /* ADC Freerun mode bit position. */
#define ADC_REFSEL_INTREF_gc (0x00<<4)
#define ADC_SAMPCAP_bp 6
#define ADC_PRESC_DIV2_gc (0x00<<0)  /* CLK_PER divided by 2 */
#define ADC_PRESC_DIV4_gc (0x01<<0)  /* CLK_PER divided by 4 */
#define ADC_PRESC_DIV8_gc (0x02<<0)  /* CLK_PER divided by 8 */
#define ADC_PRESC_DIV16_gc (0x03<<0)  /* CLK_PER divided by 16 */
#define ADC_PRESC_DIV32_gc (0x04<<0)  /* CLK_PER divided by 32 */
#define ADC_PRESC_DIV64_gc (0x05<<0)  /* CLK_PER divided by 64 */
#define ADC_PRESC_DIV128_gc (0x06<<0)  /* CLK_PER divided by 128 */
#define ADC_PRESC_DIV256_gc (0x07<<0)  /* CLK_PER divided by 256 */

typedef struct ADC_struct
{
  register8_t CTRLA;  /* Control A */
  register8_t CTRLB;  /* Control B */
  register8_t CTRLC;  /* Control C */
  register8_t CTRLD;  /* Control D */
  register8_t CTRLE;  /* Control E */
  register8_t SAMPCTRL;  /* Sample Control */
  register8_t MUXPOS;  /* Positive mux input */
  register8_t reserved_0x07;
  register8_t COMMAND;  /* Command */
  register8_t EVCTRL;  /* Event Control */
  register8_t INTCTRL;  /* Interrupt Control */
  register8_t INTFLAGS;  /* Interrupt Flags */
  register8_t DBGCTRL;  /* Debug Control */
  register8_t TEMP;  /* Temporary Data */
  register8_t reserved_0x0E;
  register8_t reserved_0x0F;
  _WORDREGISTER(RES);  /* ADC Accumulator Result */
  _WORDREGISTER(WINLT);  /* Window comparator low threshold */
  _WORDREGISTER(WINHT);  /* Window comparator high threshold */
  register8_t CALIB;  /* Calibration */
  register8_t reserved_0x17;
} ADC_t;

extern ADC_t FakeADC;
#define ADC0 (*(ADC_t *) &FakeADC)
#define ADC1 (*(ADC_t *) &FakeADC)

typedef enum ADC_RESSEL_enum
{
  ADC_RESSEL_10BIT_gc = (0x00<<2),  /* 10-bit mode */
  ADC_RESSEL_8BIT_gc = (0x01<<2),  /* 8-bit mode */
} ADC_RESSEL_t;

typedef enum ADC_MUXPOS_enum
{
  ADC_MUXPOS_AIN0_gc = (0x00<<0),  /* ADC input pin 0 */
  ADC_MUXPOS_AIN1_gc = (0x01<<0),  /* ADC input pin 1 */
  ADC_MUXPOS_AIN2_gc = (0x02<<0),  /* ADC input pin 2 */
  ADC_MUXPOS_AIN3_gc = (0x03<<0),  /* ADC input pin 3 */
  ADC_MUXPOS_AIN4_gc = (0x04<<0),  /* ADC input pin 4 */
  ADC_MUXPOS_AIN5_gc = (0x05<<0),  /* ADC input pin 5 */
  ADC_MUXPOS_AIN6_gc = (0x06<<0),  /* ADC input pin 6 */
  ADC_MUXPOS_AIN7_gc = (0x07<<0),  /* ADC input pin 7 */
  ADC_MUXPOS_AIN8_gc = (0x08<<0),  /* ADC input pin 8 */
  ADC_MUXPOS_AIN9_gc = (0x09<<0),  /* ADC input pin 9 */
  ADC_MUXPOS_AIN10_gc = (0x0A<<0),  /* ADC input pin 10 */
  ADC_MUXPOS_AIN11_gc = (0x0B<<0),  /* ADC input pin 11 */
  ADC_MUXPOS_PTC_gc = (0x1B<<0),  /* PTC/DAC2 */
  ADC_MUXPOS_DAC0_gc = (0x1C<<0),  /* DAC0/DAC0 */
  ADC_MUXPOS_INTREF_gc = (0x1D<<0),  /* Internal Ref */
  ADC_MUXPOS_TEMPSENSE_gc = (0x1E<<0),  /* Temp sensor/DAC1 */
  ADC_MUXPOS_GND_gc = (0x1F<<0),  /* GND */
} ADC_MUXPOS_t;


typedef enum PORT_ISC_enum
{
    PORT_ISC_INTDISABLE_gc = (0x00<<0),  /* Interrupt disabled but input buffer enabled */
    PORT_ISC_BOTHEDGES_gc = (0x01<<0),  /* Sense Both Edges */
    PORT_ISC_RISING_gc = (0x02<<0),  /* Sense Rising Edge */
    PORT_ISC_FALLING_gc = (0x03<<0),  /* Sense Falling Edge */
    PORT_ISC_INPUT_DISABLE_gc = (0x04<<0),  /* Digital Input Buffer disabled */
    PORT_ISC_LEVEL_gc = (0x05<<0),  /* Sense low Level */
} PORT_ISC_t;

typedef struct SIGROW_struct
{
  register8_t DEVICEID0;
  register8_t DEVICEID1;
  register8_t DEVICEID2;
} SIGROW_t;

extern SIGROW_t FakeSIGROW;
#define SIGROW (*(SIGROW_t *) &FakeSIGROW)

typedef struct TCB_struct
{
  register8_t CTRLA;  /* Control A */
  register8_t CTRLB;  /* Control Register B */
  register8_t reserved_0x02;
  register8_t reserved_0x03;
  register8_t EVCTRL;  /* Event Control */
  register8_t INTCTRL;  /* Interrupt Control */
  register8_t INTFLAGS;  /* Interrupt Flags */
  register8_t STATUS;  /* Status */
  register8_t DBGCTRL;  /* Debug Control */
  register8_t TEMP;  /* Temporary Value */
  _WORDREGISTER(CNT);  /* Count */
  _WORDREGISTER(CCMP);  /* Compare or Capture */
  register8_t reserved_0x0E;
  register8_t reserved_0x0F;
} TCB_t;

extern TCB_t FakeTCB;
#define TCB0 (*(TCB_t *) &FakeTCB)

#define TCB_CAPT_bm  0x01  /* Capture or Timeout bit mask. */
#define TCB_ENABLE_bm  0x01  /* Enable bit mask. */
#define TCB0_INT_vect

typedef enum TCB_CLKSEL_enum
{
  TCB_CLKSEL_CLKDIV1_gc = (0x00<<1),  /* CLK_PER (No Prescaling) */
  TCB_CLKSEL_CLKDIV2_gc = (0x01<<1),  /* CLK_PER/2 (From Prescaler) */
  TCB_CLKSEL_CLKTCA_gc = (0x02<<1),  /* Use Clock from TCA */
} TCB_CLKSEL_t;

#define VREF (*(VREF_t *) 0x00A0) /* Voltage reference */
/* Voltage reference */
typedef struct VREF_struct
{
    register8_t CTRLA;  /* Control A */
    register8_t CTRLB;  /* Control B */
    register8_t CTRLC;  /* Control C */
    register8_t CTRLD;  /* Control D */
} VREF_t;

/* ADC0 reference select select */
typedef enum VREF_ADC0REFSEL_enum
{
    VREF_ADC0REFSEL_0V55_gc = (0x00<<4),  /* Voltage reference at 0.55V */
    VREF_ADC0REFSEL_1V1_gc = (0x01<<4),  /* Voltage reference at 1.1V */
    VREF_ADC0REFSEL_2V5_gc = (0x02<<4),  /* Voltage reference at 2.5V */
    VREF_ADC0REFSEL_4V34_gc = (0x03<<4),  /* Voltage reference at 4.34V */
    VREF_ADC0REFSEL_1V5_gc = (0x04<<4),  /* Voltage reference at 1.5V */
} VREF_ADC0REFSEL_t;

/* ADC1 reference select select */
typedef enum VREF_ADC1REFSEL_enum
{
    VREF_ADC1REFSEL_0V55_gc = (0x00<<4),  /* Voltage reference at 0.55V */
    VREF_ADC1REFSEL_1V1_gc = (0x01<<4),  /* Voltage reference at 1.1V */
    VREF_ADC1REFSEL_2V5_gc = (0x02<<4),  /* Voltage reference at 2.5V */
    VREF_ADC1REFSEL_4V34_gc = (0x03<<4),  /* Voltage reference at 4.34V */
    VREF_ADC1REFSEL_1V5_gc = (0x04<<4),  /* Voltage reference at 1.5V */
} VREF_ADC1REFSEL_t;

typedef struct USART_struct
{
    register8_t RXDATAL;  /* Receive Data Low Byte */
    register8_t RXDATAH;  /* Receive Data High Byte */
    register8_t TXDATAL;  /* Transmit Data Low Byte */
    register8_t TXDATAH;  /* Transmit Data High Byte */
    register8_t STATUS;  /* Status */
    register8_t CTRLA;  /* Control A */
    register8_t CTRLB;  /* Control B */
    register8_t CTRLC;  /* Control C */
    _WORDREGISTER(BAUD);  /* Baud Rate */
    register8_t reserved_1[1];
    register8_t DBGCTRL;  /* Debug Control */
    register8_t EVCTRL;  /* Event Control */
    register8_t TXPLCTRL;  /* IRCOM Transmitter Pulse Length Control */
    register8_t RXPLCTRL;  /* IRCOM Receiver Pulse Length Control */
    register8_t reserved_2[1];
} USART_t;

#define USART0              (*(USART_t *) 0x0800)
#define USART_TXEN_bm  0x4
#define USART_RXEN_bm  0x80
#define USART_RXCIE_bm  0x80
#define USART_TXCIF_bm  0x40
#define USART_DREIF_bm  0x20

#define USART_DREIE_bm 0x20

#endif
