#include <util/delay.h>
#include <avr/interrupt.h>

#include "rfm69.h"
#include "rfm69_registers.h"
#include "spi.h"
#include "pins.h"
#include "uart.h"
#include "mcu.h"

RFM69* RFM69::rfm69_ptr;

// TEST defined in Makefile of tests/
/*
 * set up the isr in the main code to avoid conflicts
 * if we want to use interrupts on the same port for
 * other purposes
 * eg
 * ISR(PORTC_PORT_vect) {
 *   rf.isr();
 * }
 */
/*
#ifndef TEST
ISR(PORTC_PORT_vect) {
  // D("*");
  RFM69::rfm69_ptr->isr = 1;
  RFM69::rfm69_ptr->pin_interrupt.port->INTFLAGS |= (1<<RFM69::rfm69_ptr->pin_interrupt.pin); // clear interrupt flag
}
#endif
*/

RFM69::RFM69() {
  this->init_vars(PC3, PC4);
}

RFM69::RFM69(pins_t pin_cs, pins_t pin_interrupt) {
  this->init_vars(pin_cs, pin_interrupt);
}

void RFM69::init_vars(pins_t pin_cs, pins_t pin_interrupt) {
  this->rfm69_ptr = this;
  this->pin_cs = pin_cs;
  this->pin_interrupt = pin_interrupt;
  this->spy_mode = 0;
  this->power_level = 23;
 }

uint8_t RFM69::init(uint32_t node_id, uint8_t network_id) {
  return this->init(868, node_id, network_id);
}

uint8_t RFM69::init(uint16_t freq, uint32_t node_id, uint8_t network_id) {
  const uint8_t CONFIG[][2] = {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_00 }, // no shaping
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_55555}, // default: 4.8 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_55555},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_50000}, // default: 5KHz, (FDEV + BitRate / 2 <= 500KHz)
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_50000},

    //* 0x07 */ { REG_FRFMSB, RF_FRFMSB_433},
    //* 0x08 */ { REG_FRFMID, RF_FRFMID_433},
    //* 0x09 */ { REG_FRFLSB, RF_FRFLSB_433},

    /* 0x07 */ { REG_FRFMSB, (uint8_t) (freq==RF_315MHZ ? RF_FRFMSB_315 : (freq==RF_433MHZ ? RF_FRFMSB_433 : (freq==RF_868MHZ ? RF_FRFMSB_868 : RF_FRFMSB_915))) },
    /* 0x08 */ { REG_FRFMID, (uint8_t) (freq==RF_315MHZ ? RF_FRFMID_315 : (freq==RF_433MHZ ? RF_FRFMID_433 : (freq==RF_868MHZ ? RF_FRFMID_868 : RF_FRFMID_915))) },
    /* 0x09 */ { REG_FRFLSB, (uint8_t) (freq==RF_315MHZ ? RF_FRFLSB_315 : (freq==RF_433MHZ ? RF_FRFLSB_433 : (freq==RF_868MHZ ? RF_FRFLSB_868 : RF_FRFLSB_915))) },


    // looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    // +17dBm and +20dBm are possible on RFM69HW
    // +13dBm formula: Pout = -18 + OutputPower (with PA0 or PA1**)
    // +17dBm formula: Pout = -14 + OutputPower (with PA1 and PA2)**
    // +20dBm formula: Pout = -11 + OutputPower (with PA1 and PA2)** and high power PA settings (section 3.3.7 in datasheet)
    ///* 0x11 */ { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    ///* 0x13 */ { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, // over current protection (default is 95mA)

    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4KHz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_2 }, // (BitRate < 2 * RxBw)
    //for BR-19200: /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_3 },
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, // DIO0 is the only IRQ we're using
    /* 0x26 */ { REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF }, // DIO5 ClkOut disable for power saving
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ { REG_RSSITHRESH, 220 }, // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
    ///* 0x2D */ { REG_PREAMBLELSB, RF_PREAMBLESIZE_LSB_VALUE } // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0 },
    /* 0x2F */ { REG_SYNCVALUE1, 0x2D },      // attempt to make this compatible with sync1 byte of RFM12B lib
    /* 0x30 */ { REG_SYNCVALUE2, network_id }, // NETWORK ID
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF },
    /* 0x38 */ { REG_PAYLOADLENGTH, 66 }, // in variable length mode: the max frame size, not used in TX
    ///* 0x39 */ { REG_NODEADRS, node_id }, // turned off because we're not using address filtering
    /* 0x3C */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE }, // TX on FIFO not empty
    /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    //for BR-19200: /* 0x3D */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_NONE | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };
  spi_init();
  pins_output(&this->pin_cs, 1); // set as output
  pins_set(&this->pin_cs, 1);    // set high

  // simple check
  uint8_t version = this->read_reg(REG_VERSION);
  if (version != 0x24) {
    return 0; // failure
  }

  pins_output(&this->pin_interrupt, 0); // set interrupt pin as input (might need pull down)

  while (this->read_reg(REG_SYNCVALUE1) != 0xaa) {
    this->write_reg(REG_SYNCVALUE1, 0xaa);
  }

  while (this->read_reg(REG_SYNCVALUE1) != 0x55) {
    this->write_reg(REG_SYNCVALUE1, 0x55);
  }

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++) {
    this->write_reg(CONFIG[i][0], CONFIG[i][1]);
  }

  this->encrypt(0); // disable during init to start from a known state
  this->set_high_power();
  this->set_mode(STANDBY);
  while ((this->read_reg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00);

  // set isr for interrupt pin rising rmf_interrupt = PA5
  // TODO replace PIN5CTRL with pin
  this->pin_interrupt.port->PIN4CTRL |= PORT_ISC_RISING_gc;

  this->node_id = node_id;
  this->set_network(network_id);

  return version;
}

uint8_t RFM69::read_reg(uint8_t addr) {
  this->select();
  spi_transfer_byte(addr & 0x7F);
  uint8_t val = spi_transfer_byte(0);
  this->unselect();

  return val;
}

void RFM69::write_reg(uint8_t addr, uint8_t value) {
  this->select();
  spi_transfer_byte(addr | 0x80);
  spi_transfer_byte(value);
  this->unselect();
}

/*
 * enable encryption: encrypt("ABCDEFGHIJKLMNOP"); (must be 16 bytes)
 * disable encryption: encrypt(null) or encrypt(0)
 */
void RFM69::encrypt(const char *key) {
  this->set_mode(STANDBY);
  if (key != 0) {
    this->select();
    spi_transfer_byte(REG_AESKEY1 | 0x80);
    for (uint8_t i=0; i<16; i++) {
      spi_transfer_byte(key[i]);
    }
    this->unselect();
  }
  this->write_reg(REG_PACKETCONFIG2, (this->read_reg(REG_PACKETCONFIG2) & 0xFE) | (key ? 1:0));
}

void RFM69::set_high_power() {
  this->write_reg(REG_OCP, RF_OCP_OFF); //disable OverCurrentProtection for HW/HC
  this->set_power_level(this->power_level);
}

/*
 * TODO can we get rid of this->mode?
 */
void RFM69::set_mode(RFM69::Mode mode) {
  if (this->mode == mode) return;

  switch (mode) {
    case TX:
      this->write_reg(REG_OPMODE, (this->read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);
      this->set_high_power_regs(1);
      break;
    case RX:
      this->write_reg(REG_OPMODE, (this->read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);
      this->set_high_power_regs(0);
      break;
    case SYNTH:
      this->write_reg(REG_OPMODE, (this->read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case STANDBY:
      this->write_reg(REG_OPMODE, (this->read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case SLEEP:
      this->write_reg(REG_OPMODE, (this->read_reg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }
  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (mode == SLEEP && (this->read_reg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  this->mode = mode;
}

void RFM69::sleep() {
  this->set_mode(SLEEP);
}

void RFM69::set_network(uint8_t network) {
  this->write_reg(REG_SYNCVALUE2, network);
}

void RFM69::select() {
  pins_set(&this->pin_cs, 0);
}

void RFM69::unselect() {
  pins_set(&this->pin_cs, 1);
}

/*
 * https://lowpowerlab.com/2021/08/31/rfm69-tx-output-power-testing-library-fix/
 * level: 0-23
 * 0-15: PA1
 * 16-19: PA1+PA2
 * 20-23: PA1+PA2+HiPWr
 */
void RFM69::set_power_level(uint8_t level) {
  uint8_t pa_setting;
  if (level>23) level = 23;
  this->power_level = level;

  // enable PA1
  if (this->power_level < 16) {
    level += 16;
    pa_setting = RF_PALEVEL_PA1_ON;
  // enable PA1+PA2
  } else {
    level += this->power_level < 20 ? 10 : 8;
    pa_setting = RF_PALEVEL_PA1_ON | RF_PALEVEL_PA2_ON;
  }
  this->set_high_power_regs(1); // always call this in case we're crossing power boundaries in TX mode

  this->write_reg(REG_PALEVEL, pa_setting | level);
}

/*
 * return 0 if limit reached
 */
uint8_t RFM69::set_power_level_relative(int8_t level_change) {
  if ((this->power_level == 23 && level_change > 0) || ((this->power_level == 0 && level_change < 0))) {
    return 0;
  }

  int8_t new_level = this->power_level + level_change;
  if (new_level < 0) new_level = 0;
  if (new_level > 23) new_level = 23;
  this->set_power_level(new_level);
  return (new_level == 0 || new_level == 23) ? 0 : 1;
}

uint8_t RFM69::get_power_level() {
  return this->power_level;
}

/*
 * internal function - for HW/HCW only:
 * enables HiPower for 18-20dBm output
 * should only be used with PA1+PA2
 */
void RFM69::set_high_power_regs(uint8_t enable) {
  if (this->power_level<20) enable=false;
  this->write_reg(REG_TESTPA1, enable ? 0x5D : 0x55);
  this->write_reg(REG_TESTPA2, enable ? 0x7C : 0x70);
}

void RFM69::send_frame(uint32_t to, const void* buffer, uint8_t size, uint8_t request_ack, uint8_t send_ack) {
  // D("sending | ");
  this->set_mode(STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((this->read_reg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  if (size > RFM69_MAX_DATA_LEN) size = RFM69_MAX_DATA_LEN;

  // control byte
  uint8_t ctl_byte = 0x00;
  if (send_ack)
    ctl_byte = RFM69_CTL_SENDACK;
  else if (request_ack)
    ctl_byte = RFM69_CTL_REQACK;

  // write to FIFO
  this->select();
  spi_transfer_byte(REG_FIFO | 0x80);
  spi_transfer_byte(size + 7);
  spi_transfer_byte((uint8_t)(to >> 16));
  spi_transfer_byte((uint8_t)(to >> 8));
  spi_transfer_byte((uint8_t)to);
  spi_transfer_byte((uint8_t)(this->node_id >> 16));
  spi_transfer_byte((uint8_t)(this->node_id >> 8));
  spi_transfer_byte((uint8_t)this->node_id);
  spi_transfer_byte(ctl_byte);

  for (uint8_t i = 0; i < size; i++)
    spi_transfer_byte(((uint8_t*) buffer)[i]);
  this->unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  this->set_mode(TX);
  // TODO sometimes PACKETSENT flags are not set and blocks
  uint32_t start_tick = clock.current_tick;
  uint32_t current_tick = start_tick;
  uint32_t limit = 400; // timeout: ms*32768/8000
  while (((current_tick - start_tick) < limit) && ((this->read_reg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PACKETSENT) == 0x00)) {
    current_tick = clock.current_tick;
  }
  this->set_mode(STANDBY);
  // DF("sent (%lu) | ", current_tick-start_tick);
}

uint8_t RFM69::send(uint32_t to, const void* buffer, uint8_t buffer_len, RFM69::Packet *response) {
  this->write_reg(REG_PACKETCONFIG2, (this->read_reg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  // we passed a response object, so do ack communication and wait for answer
  this->send_frame(to, buffer, buffer_len, response ? 1 : 0, 0);
  if (response) return this->listen(response);

  return 1;
}

uint8_t RFM69::send_retry(uint32_t to, const void* buffer, uint8_t buffer_len, RFM69::Packet *response, uint8_t retries) {
  for (uint8_t i=0; i<retries; i++) {
    if (!i) D("  ");
    if (this->send(to, buffer, buffer_len, response)) {
      if (i) DL("") else { uart_send_char(0x08);uart_send_char(0x08); } // remove 2 spaces printed out on send()
      return 1;
    }

    this->set_mode(STANDBY);
    DF(WARN("retry %u (%idBm)") " | ", i, this->read_rssi());
    clock.sleep_for(410); // timeout until retry ms*32768/8000
  }

  return 0;
}

uint8_t RFM69::listen(RFM69::Packet *response, uint8_t timeout_enabled) {
  if (this->read_reg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
    this->write_reg(REG_PACKETCONFIG2, (this->read_reg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks

  this->write_reg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01); // set DIO0 to "PAYLOADREADY" in receive mode

  this->set_mode(RX);
  this->isr = 0;
  uint32_t start_tick = clock.current_tick;
  uint32_t current_tick = start_tick;
  uint32_t limit = 300; // timeout: ms*32768/8000
  if (timeout_enabled) {
    while (((current_tick - start_tick) < limit) && !this->isr) {
      current_tick = clock.current_tick;
    }
  } else {
    while (!this->isr);
  }
  // DF("%lu ticks | ", current_tick-start_tick);

  if (!(this->read_reg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
    DF(NOK("no data (%lu)") " | ", current_tick-start_tick);
    return 0;
  }

  // we've got some data ready
  this->set_mode(STANDBY);
  this->select();
  spi_transfer_byte(REG_FIFO & 0x7F);
  uint8_t payload_len = spi_transfer_byte(0);
  payload_len = payload_len > 66 ? 66 : payload_len;
  uint32_t to = ((uint32_t)spi_transfer_byte(0) << 16) | ((uint16_t)spi_transfer_byte(0) << 8) | spi_transfer_byte(0);
  if (!(spy_mode || to == this->node_id || to == RFM69_BROADCAST_ADDR) || payload_len < 7) {
    D("not for us | ");
    this->unselect();
    return 0;
  }

  response->from = ((uint32_t)spi_transfer_byte(0) << 16) | ((uint16_t)spi_transfer_byte(0) << 8) | spi_transfer_byte(0);
  uint8_t ctl_byte = spi_transfer_byte(0);
  uint8_t ack_received = ctl_byte & RFM69_CTL_SENDACK; // extract ACK-received flag
  uint8_t ack_requested = ctl_byte & RFM69_CTL_REQACK; // extract ACK-requested flag

  // read data
  uint8_t datalen = payload_len - 7; // uint8_t len, uint24_t to, uint24t from, uint8_t ctl
  response->len = datalen;
  for (uint8_t i = 0; i < datalen; i++) {
      response->payload[i] = spi_transfer_byte(0);
  }
  if (datalen < RFM69_MAX_DATA_LEN) response->payload[datalen] = 0; // add null at end of string
  this->unselect();

  response->rssi = this->read_rssi();

  // send ack. avoid ping-poing so we dont ack an ack request
  if (ack_requested && !ack_received) {
    this->send_frame(response->from, "", 0, 0, 1);
  }

  // DF("data from %u: '%s' (%lu)\n", response->from, response->payload, current_tick-start_tick);
  return 1;
 }

int16_t RFM69::read_rssi(uint8_t force) {
  int16_t rssi = 0;
  if (force) {
    // RSSI trigger not needed if DAGC is in continuous mode
    this->write_reg(REG_RSSICONFIG, RF_RSSI_START);
    while ((this->read_reg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00); // wait for RSSI_Ready
  }
  rssi = -this->read_reg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}
