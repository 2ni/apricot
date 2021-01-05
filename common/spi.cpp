#include <avr/io.h>
#include "spi.h"
#include "pins.h"

/*
 * ok values: 8MHz, msb first, mode0 (alternative: mode3)
 */
void spi_init(uint8_t mode) {
/*
1. Configure the SS pin in the port peripheral.
2. Select SPI Master/Slave operation by writing the Master/Slave Select bit (MASTER) in the Control A register (SPIn.CTRLA).
3. In Master mode, select the clock speed by writing the Prescaler bits (PRESC) and the Clock Double bit (CLK2X) in SPIn.CTRLA.
4. Optional: Select the Data Transfer mode by writing to the MODE bits in the Control B register (SPIn.CTRLB).
5. Optional: Write the Data Order bit (DORD) in SPIn.CTRLA.
6. Optional: Setup Buffer mode by writing BUFEN and BUFWR bits in the Control B register (SPIn.CTRLB).
7. Optional: To disable the multi-master support in Master mode, write ‘1’ to the Slave Select Disable bit (SSD) in SPIn.CTRLB.
8. Enable the SPI by writing a ‘1’ to the ENABLE bit in SPIn.CTRLA.
*/

  // SS probably doesn't need to be set as output to avoid spi going highwire, see SPI_SSD_bm
  pins_output(&pins_mosi, 1); // mosi as output
  pins_output(&pins_sck, 1);  // sck as output
  pins_output(&pins_miso, 0); // miso as input

  SPI0.CTRLB = SPI_SSD_bm | mode; // ignore SS pin setting for master, set spi mode 0

  SPI0.CTRLA = (0<<SPI_DORD_bp) | SPI_ENABLE_bm | SPI_MASTER_bm; // | SPI_PRESC_DIV4_gc; // MSB first, master mode, enable spi, no double clk, prescaler
  SPI0.DATA; // empty rx
}

/*
 * send / receive data in full-duplex
 */
void spi_transfer (uint8_t *dataout, uint8_t *datain, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    SPI0.DATA = dataout[i];

    while (!(SPI0.INTFLAGS & SPI_IF_bm));
    datain[i] =  SPI0.DATA;
  }
}

/*
 * send data only, ignore receiving data
 */
void spi_send (uint8_t *dataout, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    SPI0.DATA = dataout[i];

    while (!(SPI0.INTFLAGS & SPI_IF_bm));
  }
}

/*
 * send / receive  1 byte full-duplex
 */
uint8_t spi_transfer_byte (uint8_t data) {
  SPI0.DATA = data;

  while (!(SPI0.INTFLAGS & SPI_IF_bm));
  return SPI0.DATA;
}
