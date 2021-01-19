#ifndef __TWI_H__
#define __TWI_H__

#include <avr/io.h>

#define TWI_BAUD(F_SCL)  ((((float)F_CPU / (float)F_SCL)) - 10 )
#define TWI_MAXLEN        8

void    twi_init();
uint8_t twi_wait_ack();
uint8_t twi_error(uint8_t status);
void    twi_stop();
uint8_t twi_start(uint8_t addr, uint8_t read = 0);
uint8_t twi_read(uint8_t *data, uint8_t ack);
uint8_t twi_write(uint8_t data);
uint8_t twi_write_bytes(uint8_t device_addr, uint8_t *data, uint8_t slave_reg, uint8_t num_bytes);
uint8_t twi_read_bytes(uint8_t device_addr, uint8_t *data, uint8_t slave_reg, uint8_t num_bytes, uint8_t wait_for_ack = 0);

#endif
