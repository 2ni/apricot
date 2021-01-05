/*
 * http://ww1.microchip.com/downloads/en/AppNotes/TB3215-Getting-Started-with-SPI-90003215A.pdf
 * https://github.com/MicrochipTech/TB3215_Getting_Started_with_SPI
 */

#include <avr/io.h>

#ifndef SPI_h
#define SPI_h

void    spi_init(uint8_t mode=0);
void    spi_transfer(uint8_t *dataout, uint8_t *datain, uint8_t len);
void    spi_send(uint8_t *dataout, uint8_t len);
uint8_t spi_transfer_byte(uint8_t data);

#endif
