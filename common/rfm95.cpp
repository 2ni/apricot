/*
 * inspired by https://www.thethingsnetwork.org/forum/t/attiny85-rfm95-temperature-sensor/11211
 *
 * see lorawan spec https://lora-alliance.org/resource-hub/lorawanr-specification-v103
 *
 * https://cdn.hackaday.io/files/286681226531712/RH_RF95.cpp
 * ??? TODO remember last SNR:
 * lastSNR = (int8_t)read_reg(0x1a) / 4
 *
 * ??? TODO remember last RSSI
 * lastRssi = read_reg(0x1a);
 *  // Adjust the RSSI, datasheet page 87
 *  if (lastSNR < 0)
 *      lastRssi = lastRssi + lastSNR;
 *  else
 *      lastRssi = (int)lastRssi * 16 / 15;
 *  if (_usingHFport)
 *      _lastRssi -= 157;
 *  else
 *      _lastRssi -= 164;
 *
 *  PC0 = SCK
 *  PC1 = MISO
 *  PC2 = MOSI
 *  PC3 = CS_RFM
 *  PC4 = DIO0
 *  PC5 = DIO1
 */

#include <util/delay.h>

#include "rfm95.h"
#include "spi.h"
#include "sleep.h"
#include "pins.h"
#include "uart.h"

// RFM95::RFM95(pins_t *ics, pins_t *idio0, pins_t *idio1) {
RFM95::RFM95(pins_t *ics) {
  cs = ics;
  // dio0 = idio0;
  // dio1 = idio1;
}

uint8_t RFM95::init() {
  spi_init();
  pins_output(cs, 1); // set as output
  pins_set(cs, 1);    // set high (spi bus disabled by default)

  // spi communication should only start after 10ms (p.108)
  sleep_ms(10);

  // pins_output(dio0, 0); // input, DIO0
  // pins_output(dio1, 0); // input, DIO1

  // reset might be an issue
  uint8_t version = read_reg(0x42);
  if (version != 0x11 && version != 0x12) { // some inofficial chips return ox12
    return 0; // failure
  }

  // sleep to switch to lora mode
  write_reg(0x01, 0x00);
  write_reg(0x01, 0x80);

  // ensure we successfully switched to lora
  if (read_reg(0x01) != 0x80) {
    return 0;
  }

  // set IQ (documented in AN1200.24)
  write_reg(0x33, 0x27);
  write_reg(0x3B, 0x1D);

  // set max packet len to 65 (documented in AN1200.24)
  write_reg(0x23, 0x40);

  // set fifo pointers
  // tx base address
  write_reg(0x0E, 0x80);
  // rx base adress
  write_reg(0x0F, 0x00);

  // rx timeout set to 37 symbols
  write_reg(0x1F, 0x25); // according to AN1200.24: 0x05 if sf>10 else 0x08

  // preamble length set to 8 symbols
  // 0x0008 + 4 = 12
  write_reg(0x20, 0x00);
  write_reg(0x21, 0x08);

  // set lora sync word (0x34)
  write_reg(0x39, 0x34);

  // set carrier frequency
  set_frq(8681000);

  // SW12, 125kHz
  set_datarate(12);

  // disable current protection
  // write_reg(0x0b, 0x3f);

  set_power(12);
  // DF("regpaconfig: 0x%02x\n", read_reg(0x09));

  set_mode(0); // sleep

  // DT_IH("reg", read_reg(0x01));

  return version;
}

/*
 * enable spi transfer
 */
void RFM95::select() {
  pins_set(cs, 0);
}

/*
 * disable spi transfer
 */
void RFM95::unselect() {
  pins_set(cs, 1);
}

uint8_t RFM95::read_reg(uint8_t addr) {
  select();
  spi_transfer_byte(addr & 0x7F);
  uint8_t val = spi_transfer_byte(0);
  unselect();

  return val;
}

void RFM95::write_reg(uint8_t addr, uint8_t value) {
  select();
  spi_transfer_byte(addr | 0x80);
  spi_transfer_byte(value);
  unselect();
}

/*
 * receive on 869.525, SF9
 * (directly opening window RX2)
 *
 */
void RFM95::receive_continuous(uint32_t frq, uint8_t datarate) {
  // set iq (documented in AN1200.24
  write_reg(0x33, 0x67);
  write_reg(0x3B, 0x19);

  set_mode(1); // standby

  set_frq(frq);
  set_datarate(datarate); // eg 12 for SF12. SF9 for "normal", SF12 for join accept in rx2

  // set hop frequency period to 0
  write_reg(0x24, 0x00);

  set_mode(5); // continuous rx
  DF("listening on %lu SF%u\n", frq, datarate);
}

/*
 * only get one package within a tight timeline
 * and times out if nothing received
 *
 * rx1 with same channel, datarate as tx
 * rx2 with 869.525, SF9
 *
 * 0: no errors (no timeout)
 * 1: timeout
 */
Status RFM95::wait_for_single_package(uint32_t frq, uint8_t datarate) {
  set_mode(1);

  write_reg(0x40, 0x00); // DIO0 -> rxdone

  // set iq (documented in AN1200.24)
  write_reg(0x33, 0x67);
  write_reg(0x3B, 0x19);

  write_reg(0x0c, 0x23); // AN1200.24

  // set downlink carrier frq
  set_frq(frq);
  set_datarate(datarate);

  set_mode(6); // single rx

  // wait for rx done or timeout
  uint8_t f = 0;
  while (1) {
    f = read_reg(0x12);
    if ((f & 0x40) || (f & 0x80)) {
      break;
    }
  }
  // DT_IH("reg", f);

  write_reg(0x12, 0xff); // clear interrupt
  if (f & 0x80) {
    set_mode(0);
    return TIMEOUT;
  } else {
    return OK;
  }
}

/*
 * get PHYPayload
 * return error code
 * 0: no error
 * 1: crc error
 */
Status RFM95::read(Packet *packet) {
  uint8_t interrupts = read_reg(0x12);
  Status status = OK;

  if ((interrupts & 0x20)) {
    status = ERROR; // PayloadCrcError set, we've got an error
  }

  // read rfm package
  uint8_t start_pos = read_reg(0x10);
  packet->len = read_reg(0x13);
  write_reg(0x0D, start_pos);
  for (uint8_t i=0; i<packet->len; i++) {
    packet->data[i] = read_reg(0x00);
  }

  write_reg(0x12, 0xff); // clear interrupt register
  set_mode(0); // sleep

  return status;
}

/*
 * TODO
 * fallback if tx timeout: https://github.com/Lora-net/LoRaMac-node/blob/develop/src/radio/sx1276/sx1276.c#L1564
 *
 * improvements (from the comments):
 * if you would like to use a semi-random key to select the channel,
 * just take a byte from the encrypted payload (or a byte from the MIC)
 * and mask-out the 3 least significant bits as a pointer to the channel
 */
void RFM95::send(const Packet *packet, const uint32_t frq, const uint8_t datarate, const uint8_t power) {
  set_mode(1); // standby

  write_reg(0x40, 0x40); // DIO0 -> txdone

  // iq to standard values according to AN1200.24
  write_reg(0x33, 0x27);
  write_reg(0x3B, 0x1D);

  write_reg(0x0a, 0x08); // 50us PA ramp-up time (documented in AN1200.24)

  set_power(power);
  set_frq(frq);
  set_datarate(datarate); // eg 12 for SF12

  write_reg(0x12, 0xff); // clear interrupt
  write_reg(0x22, packet->len); // set length

  uint8_t tx_pos = read_reg(0x0E);
  write_reg(0x0D, tx_pos);
  // write_reg(0x0D, 0x80); // hardcoded fifo location according RFM95 specs

  // write payload to fifo
  for (uint8_t i = 0; i<packet->len; i++) {
    write_reg(0x00, packet->data[i]);
  }

  // DL("rfm95 sending...");
  if (set_mode(3) == 0) { // tx
    while (!(read_reg(0x12) & 0x08)) {
      // DF("irqflags: 0x%02x, status: %u\n", read_reg(0x12), read_reg(0x01) & 0x07);
    }
    //while(pins_get(dio0) == 0) {} // wait for txdone
  } else {
    DL(NOK("set tx mode failed"));
  }
  // DF("irqflags: 0x%02x, status: %u\n", read_reg(0x12), read_reg(0x01) & 0x07);
  // DL("done");

  write_reg(0x12, 0xff); // clear interrupt

  set_mode(0); // sleep

  // uart_arr(WARN("pkg sent"), packet->data, packet->len);
}

/*
 * mode considers only 3 low bits
 * 0 = sleep
 * 1 = standby
 * 2 = fs mode tx (FSTx)
 * 3 = transmitter mode (Tx)
 * 4 = fs mode rx (FSRx)
 * 5 = receiver continuous
 * 6 = receiver single
 * 7 = channel activity detection
 */
uint8_t RFM95::set_mode(uint8_t mode) {
  // write_reg(0x01, (read_reg(0x01) & ~0x07) | mode);
  write_reg(0x01, 0x80 | mode);
  // wait for mode switched
  while ((read_reg(0x01) & 0x07) != mode) {
    _delay_us(1);
  };
  return 0;

  // TODO might need to have a fallback for 0x03 (tx) which can hang sometimes
  /*
  if ((read_reg(0x01) & 0x07) != mode) {
    // attempt to reset device
    DF(NOK("mode switch failed: %03x -> %03x") "\n", read_reg(0x01) & 0x07, mode);
    write_reg(0x01, 0x00);
    sleep_ms(1);
    write_reg(0x01, 0x80);
    sleep_ms(1);
    uint8_t i=0;
    while (i<20) {
      write_reg(0x01, 0x80 | mode);
      sleep_ms(10);
      if ((read_reg(0x01) & 0x07) != mode) break;
      i++;
    }
    return 1;
  }
  return 0;
  */
}

/*
 * set power 5-23dBm
 * PA_BOOST is used on RFM95
 *
 * based on
 * - https://github.com/adafruit/RadioHead/blob/master/RH_RF95.cpp#L392
 * - https://github.com/dragino/RadioHead/blob/master/RH_RF95.cpp#L367
 * - https://github.com/Lora-net/LoRaMac-node
 *
 */
void RFM95::set_power(int8_t power) {
  if (power > 20) power = 20;
  if (power < 2) power = 2;

  if (power > 17) {
    write_reg(0x4d, 0x07); // RH_RF95_PA_DAC_ENABLE: adds ~3dBm to all power levels, use it for 21, 22, 23dBm
    power -= 3;
  } else {
    write_reg(0x4d, 0x04); // RH_RF95_PA_DAC_DISABLE
  }

  // RegPAConfig MSB = 1 to choose PA_BOOST
  write_reg(0x09, 0x80 | (power-2));
}

/*
 * based on
 * https://github.com/ErichStyger/mcuoneclipse/blob/master/Examples/KDS/tinyK20/LoRaMac-node/src/radio/sx1276/sx1276.c
 *
 */
uint32_t RFM95::get_random(uint8_t bits) {
  uint32_t rnd = 0x0000;

  // disable interrupts
  // write_reg(0x11, 0x80 | 0x40 | 0x20 | 0x10 | 0x08 | 0x04 | 0x02 | 0x01);
  // DF("interrupts: 0x%02x\n", read_reg(0x11));

  set_mode(5); // continuous receive

  for (uint8_t i=0; i<bits; i++) {
    _delay_us(100);
    rnd |= ((uint32_t)read_reg(0x2C) & 0x01 ) << i; // unfiltered RSSI value reading (lsb value)
  }

  set_mode(0); // sleep

  return rnd;
}

/*
 * https://www.thethingsnetwork.org/docs/lorawan/frequency-plans.html
 */
void RFM95::set_datarate(uint8_t rate) {
  switch(rate) {
    case 12: // SF12 BW 125 kHz (DR0)
      write_reg(0x1E, 0xC4); // SF12 CRC On
      write_reg(0x1D, 0x72); // 125 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x0C); // Low datarate optimization on AGC auto on
      break;
    case 11: // SF11 BW 125 kHz (DR1)
      write_reg(0x1E, 0xB4); // SF11 CRC On
      write_reg(0x1D, 0x72); // 125 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x0C); // Low datarate optimization on AGC auto on
      break;
    case 10: // SF10 BW 125 kHz (DR2)
      write_reg(0x1E, 0xA4); // SF10 CRC On
      write_reg(0x1D, 0x72); // 125 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x04); // Low datarate optimization off AGC auto on
      break;
    case 9: // SF9 BW 125 kHz (DR3)
      write_reg(0x1E, 0x94); // SF9 CRC On
      write_reg(0x1D, 0x72); // 125 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x04); // Low datarate optimization off AGC auto on
    break;
    case 8: // SF8 BW 125 kHz (DR4)
      write_reg(0x1E, 0x84); // SF8 CRC On
      write_reg(0x1D, 0x72); // 125 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x04); // Low datarate optimization off AGC auto on
    break;
    case 7: // SF7 BW 125 kHz (DR5)
      write_reg(0x1E, 0x74); // SF7 CRC On
      write_reg(0x1D, 0x72); // 125 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x04); // Low datarate optimization off AGC auto on
    break;
    /*
    case 72: // SF7 BW 250kHz (DR6)
      write_reg(0x1E, 0x74); // SF7 CRC On
      write_reg(0x1D, 0x82); // 250 kHz 4/5 coding rate explicit header mode
      write_reg(0x26, 0x04); // Low datarate optimization off AGC auto on
    break;
    */
  }
}

/*
 * set frequency of transmitter
 * in kHz to spare some 0's
 * registers = frq*2^19/32MHz = frq * 0.016384 = frq / 61.03515625
 *
 * eg 8681000 or 8695250
 */
void RFM95::set_frq(uint32_t frq) {
  uint32_t frf = (uint64_t)frq*524288/320000;
  // DF("frf: 0x%lx\n", frf);
  // DF("frq: %lu\n", frq);
  write_reg(0x06, (uint8_t)(frf>>16) & 0xff);
  write_reg(0x07, (uint8_t)(frf>>8) & 0xff);
  write_reg(0x08, (uint8_t)(frf>>0));
}

/*
 *
 * https://www.thethingsnetwork.org/docs/lorawan/frequency-plans.html
 * default channels 0..2: 868.1, 868.3, 868.5
 * only changeable in sleep or standby
 * regs = 2^19*frq/32MHz
 *
 */
void RFM95::set_channel(uint8_t channel) {
  if (channel == 99) set_frq(FREQUENCY_UP);
  else set_frq(FREQUENCIES[channel]);
}
