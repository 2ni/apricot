#include <util/delay.h>
#include <avr/io.h>
#include "twi.h"
#include "pins.h"

/*
 * default values:
 * SCL = PB0
 * SDA = PB1
 */
void twi_init() {
  pins_pullup(&pins_scl, 1);
  pins_pullup(&pins_sda, 1);
  pins_output(&pins_scl, 1);
  pins_output(&pins_sda, 1);

  TWI0.MBAUD = (uint8_t)TWI_BAUD(400000);
  TWI0.MCTRLB = TWI_FLUSH_bm;
  TWI0.MCTRLA = TWI_ENABLE_bm;
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
  TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm | TWI_BUSERR_bm);
}

uint8_t twi_wait_ack(void) {
  uint8_t timeout_cnt = 0;
  while (!(TWI0.MSTATUS & TWI_RIF_bm) && !(TWI0.MSTATUS & TWI_WIF_bm)) {
    if (timeout_cnt++ == 255) return 0xff;
  }
  TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
  if (TWI0.MSTATUS & TWI_BUSERR_bm) return 4;
  if (TWI0.MSTATUS & TWI_ARBLOST_bm) return 2;
  if (TWI0.MSTATUS & TWI_RXACK_bm) return 1;
  return 0;
}

uint8_t twi_error(uint8_t status) {
  // twi_recover();
  // return 0xff;
  return status;
}

void twi_stop() {
  TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
}


/*
 * device_addr = 7bit address of i2c sensor
 */
uint8_t twi_start(uint8_t device_addr) {
  uint8_t status;
  TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
  if (TWI0.MSTATUS & TWI_BUSERR_bm) return twi_error(4);
  TWI0.MADDR = device_addr<<1;

  status = twi_wait_ack();
  if (status != 0) {
    if (status == 1) twi_stop();
    return status;
  }

  return 0;
}

/*
 * ack_flag = 0 -> send ACK
 * ack_flag = 1 -> send NACK
 */
uint8_t twi_read(uint8_t *data, uint8_t ack_flag) {
  uint16_t timeout_cnt = 0;
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc) {
    while (!(TWI0.MSTATUS & TWI_RIF_bm)) {
      if (++timeout_cnt == 65535) return 0xff;
      _delay_us(2); // TODO
    }
    TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
    if (TWI0.MSTATUS & TWI_BUSERR_bm) return 4;
    if (TWI0.MSTATUS & TWI_ARBLOST_bm) return 2;
    if (TWI0.MSTATUS & TWI_RXACK_bm) return 1;
    if (ack_flag == 0) TWI0.MCTRLB &= ~(1 << TWI_ACKACT_bp);
    else            TWI0.MCTRLB |= TWI_ACKACT_NACK_gc;
    *data = TWI0.MDATA;
    if (ack_flag == 0) TWI0.MCTRLB |= TWI_MCMD_RECVTRANS_gc;
    return 0;
  }
  else return 8;
}

uint8_t twi_write(uint8_t data) {
  uint16_t timeout_cnt = 0;
  if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc) {
    TWI0.MDATA = data;
    while (!(TWI0.MSTATUS & TWI_WIF_bm)) {
      if (timeout_cnt++ == 65535) return 0xff;
    }
    if (TWI0.MSTATUS & TWI_BUSERR_bm) return 4;
    if (TWI0.MSTATUS & TWI_RXACK_bm) return 1;
    return 0;
  }
  else return 8;
}

uint8_t twi_write_bytes(uint8_t device_addr, uint8_t *data, uint8_t slave_reg, uint8_t num_bytes) {
    uint8_t status;
    if (num_bytes > TWI_MAXLEN) num_bytes = TWI_MAXLEN;

    status = twi_start(device_addr); // wait for slave ACK
    if (status != 0) return status;

    status = twi_write(slave_reg);
    if (status != 0) return twi_error(status);
    while (num_bytes > 0) {
      status = twi_write(*data);
      if (status != 0) return twi_error(status);
      data++;
      num_bytes--;
    }
    twi_stop();
    return 0;
}

/*
 * device_addr = 7bit address of i2c sensor
 */
uint8_t twi_read_bytes(uint8_t device_addr, uint8_t *data, uint8_t slave_reg, uint8_t num_bytes) {
  uint8_t status;
  if (num_bytes > TWI_MAXLEN) num_bytes = TWI_MAXLEN;

  status = twi_start(device_addr); // wait for slave ACK
  if (status != 0) return status;

  status = twi_write(slave_reg);
  if (status != 0) return twi_error(status);
  // DF("status 4: %u\n", status);
  TWI0.MADDR = (device_addr<<1) + 1;
  while (num_bytes > 1) {
    status = twi_read(data, 0); // first bytes, send ACK
    // DF("status 5: %u\n", status);
    if (status != 0) return twi_error(status);
    data++;
    num_bytes--;
  }
  status = twi_read(data, 1); // single or last byte, send NACK
  // DF("status 6: %u\n", status);
  if (status != 0) return twi_error(status);
  twi_stop();
  return 0;
}
