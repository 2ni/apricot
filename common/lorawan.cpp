/*
 * resources:
 * https://github.com/matthijskooijman/arduino-lmic
 * https://lora-developers.semtech.com/library/tech-papers-and-guides/the-book/joining-and-rejoining/
 * https://www.alanzucconi.com/2017/03/13/positioning-and-trilateration/
 * https://hackmd.io/@hVCY-lCeTGeM0rEcouirxQ/S1kg6Ymo-?type=view
 *
 * https://eu1.cloud.thethings.network/
 * https://www.thethingsindustries.com/docs/download/
 * https://www.thethingsnetwork.org/docs/the-things-stack/index.html
 * https://www.thethingsindustries.com/docs/gateways/multitechconduit/
 * https://www.thethingsindustries.com/docs/gateways/multitechconduit/lbs/
 *
 *
 *
 * TODOs
 * - make it work on ttn v3
 * - adaptive mode
 * - check cflist 8 vs 16 bit (downlink [8] should not be overwritten!
 * - rx window should start listening after 1sec + 1.5-2 symbols (symbol depending on datarate)
 * - maybe shorten timeout settings of rfm95
 * - packet with fixed max length data instead of pointer
 *
 */

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "string.h"

#include "lorawan.h"
#include "lorawan_struct.h"
#include "aes.h"
#include "cmac.h"
#include "uart.h"
#include "rfm95.h"
#include "clock.h"
#include "mcu.h"

/*
 * otaa deveui, appeui, appkey defined in keys.h -> generate devaddr, nwkskey, appskey
 * abp devaddr, nwkskey, appskey defined in keys.h
 *
 * otaa
 * LORAWAN lora(cs, dio0, dio1);
 * LORAWAN lora();
 * lora.set_otaa(deveui, appeui, appkey);
 * lora.join();
 * lora.send();
 *
 * abp
 * LORAWAN lora(cs, dio0, dio1);
 * LORAWAN lora();
 *  lora.set_abp(devaddr, nwkskey, appskey);
 * lora.send();
 *
 */

/*
 * check the link every 2^check_link_nth package sent
 * when ADR these defaults are set:
 * - max allowed tx power
 * - data rate from join or minimum (SF12)
 *
 * if datarate > proposed datarate || txpower < proposed txpower or to check connectivity:
 * set ADRACKReq after n transmissiosn -> GW must respond. If received -> clear ADRACKReq
 * if not received -> set txpower to default, then switch to next lower data rate
 * at the end rejoin
 */
LORAWAN::LORAWAN(uint8_t i_adaptive, uint8_t i_check_link_nth) {
  adaptive = i_adaptive ? 1 : 0;
  check_link_nth = i_check_link_nth;
}

void LORAWAN::set_otaa(uint8_t *deveui, uint8_t *appeui, uint8_t *appkey) {
  aes128_copy_array(otaa.deveui, deveui, 8);
  aes128_copy_array(otaa.appeui, appeui, 8);
  aes128_copy_array(otaa.appkey, appkey, 16);
}

void LORAWAN::set_abp(uint8_t *devaddr, uint8_t *nwkskey, uint8_t *appskey, const uint16_t counter) {
  aes128_copy_array(session.devaddr, devaddr, 4);
  aes128_copy_array(session.nwkskey, nwkskey, 16);
  aes128_copy_array(session.appskey, appskey, 16);
  session.counter = counter;
}

void LORAWAN::_set_persistent(Lora_session *i_ee_session) {
  persistent = 1;
  ee_session = i_ee_session;

  // read session from eeprom and keep if valid
  eeprom_read_block(&session, ee_session, sizeof(session));
  if (!has_session()) {
    DL("no eeprom session");
    reset_session();
  }
}

void LORAWAN::print_session() {
  uart_arr("nwkskey", session.nwkskey, 16);
  uart_arr("appskey", session.appskey, 16);
  uart_arr("devaddr", session.devaddr, 4);
  DF("counter      : %u\n", session.counter);
  DF("tx datarate  : %u\n", session.txdatarate);
  DF("rx delay     : %u\n", session.rxdelay);
  DF("rx offset    : %u\n", session.rxoffset);
  DF("rx2 datarate : %u\n", session.rx2datarate);
  DF("tx power     : %u\n", session.txpower);
  DF("chmask       : 0x%02x\n", session.chmask);
  D("frequencies  : ");
  for (uint8_t i=0; i<8; i++) {
    if ((session.chmask>>i) & 0x01) {
      DF("%lu, ", session.frequencies[i]);
    }
  }
  DL("");
}

void LORAWAN::_persist_session() {
  if (persistent) {
    DL("persisting session");
    print_session();
    eeprom_update_block(&session, ee_session, sizeof(session));
  }
}

uint8_t LORAWAN::has_session() {
  return session.txdatarate != 0xff;
}

/*
 * resets settings to defaults
 * TODO session.chmask is uint_16t but session.frequencies[8]
 * use sessoin.chmask / session.frequencies while sending
 */
void LORAWAN::reset_session() {
  /*
  uint8_t f[16] = { 0xF0, 0x99, 0x27, 0xA6, 0x77, 0x4E, 0x47, 0x35, 0x5D, 0xDE, 0x94, 0x43, 0xC5, 0x15, 0x17, 0xAB };
  for (uint8_t i=0; i<16; i++) session.nwkskey[i] = f[i];
  uint8_t g[16] = { 0xBC, 0x31, 0x2D, 0x42, 0xEF, 0x25, 0x5B, 0x0A, 0x2A, 0xD2, 0xAC, 0xFE, 0xED, 0x88, 0x5E, 0x66 };
  for (uint8_t i=0; i<16; i++) session.appskey[i] = g[i];
  uint8_t h[4] = {0x26, 0x01, 0x42, 0xd1};
  for (uint8_t i=0; i<4; i++) session.devaddr[i] = h[i];
  */

  for (uint8_t i=0; i<16; i++) {
    session.nwkskey[i] = 0;
    session.appskey[i] = 0;
  }

  for (uint8_t i=0; i<4; i++) {
    session.devaddr[i] = 0;
  }

  session.counter = 0;
  session.txdatarate = 0xff; // invalid session
  session.rxdelay = 1;
  session.rxoffset = 0;
  session.rx2datarate = 9;
  session.txpower = 16;
  session.chmask = 0x07; // only 8681000, 8683000, 8685000 by default
  chmask_current_pos = 0; // we start at 8681000

  for (uint8_t i=0; i<8; i++) {
    session.frequencies[i] = 0;
  }
  session.frequencies[0] = 8681000;
  session.frequencies[1] = 8683000;
  session.frequencies[2] = 8685000;
  session.frequencies[8] = 8695250; // downlink
}

/*
 * to avoid tests blocking, this needs to be in a separate function
 * which must be called in the real programm but not in the test suite
 * can't group it because pointer to EEMEM always returns NULL
 */
uint8_t LORAWAN::init(pins_t *cs) {
  // RFM95 rfm95(cs, dio0, dio1);
  RFM95 rfm95(cs);

  reset_session();

  uint8_t version = rfm95.init();
  if (!version) {
    DL(NOK("RFM95 init failed."));
    return ERROR;
  } else {
    DF("RFM95 version: 0x%02x\n", version);
    return OK;
  }
}

uint8_t LORAWAN::init(Lora_session *i_ee_session) {
  uint8_t status = init(&pins_csrfm);
  _set_persistent(i_ee_session);
  return status;
}

uint8_t LORAWAN::init(pins_t *cs, Lora_session *i_ee_session) {
  uint8_t status = init(&pins_csrfm);
  _set_persistent(i_ee_session);
  return status;
}

/*
 * https://lora-alliance.org/sites/default/files/2020-06/rp_2-1.0.1.pdf
 * RECEIVE_DELAY1 1s
 * RECEIVE_DELAY2 2s (RECEIVE_DELAY1 + 1s)
 *
 * JOIN_ACCEPT_DELAY1 5sec
 * JOIN_ACCEPT_DELAY2 6sec (869.525MHz, SF12)
 *
 * SF7 and SF8 should have a join reply in RX1 using the parameters of the request if gw has available airtime
 * SF9+ response is in RX2 at SF12
 *
 */


/*
 * return 0: no errors
 *
 * tries to join several time by lowering the datarate each time
 *
 * TODO handle sending power (rssi)
 *
 */
Status LORAWAN::join(uint8_t wholescan) {
  reset_session();
  _persist_session(); // invalidate session (txdatarate = 0xff with reset_session)

  uint8_t tx_frq_pos = 0;
  uint8_t tx_datarate = 7;

  uint8_t rx_frq_pos;
  uint8_t rx_datarate;

  uint8_t valid_lora;
  uint8_t trials = 0;
  uint16_t num_trials = 7;

  // we try multiple times to get a join accept package (trying to fullfill retransmission on p.65 of 1.0.3 specification)
  while (trials < num_trials) {
    valid_lora = 0;
    otaa.devnonce = rfm95.get_random();
    tx_frq_pos = get_next_frq_pos();
    rx_frq_pos = tx_frq_pos;
    rx_datarate = tx_datarate;

    // use correct datarates, frequencies for join request and accept listening
    // see "receive windows" in https://lora-alliance.org/sites/default/files/2018-05/lorawan_regional_parameters_v1.0.2_final_1944_1.pdf
    // (RX1DROffset: 0)
    // join accept SF7-SF11 -> RX1 after 5sec: SF7-SF12
    // join accept SF7-SF12 -> RX2 after 6sec: SF12, 869.525MHz

    // create join request packet and send it
    uint8_t len = 23; // mhrd(1) + appeui(8) + deveui(8) + devnonce(2) + mic(4)
    uint8_t data[len];
    memset(data, 0, len);
    Packet join = { .data=data, .len=len };

    join.data[0] = 0x00; // mac_header (mtype 000: join request)

    for (uint8_t i=0; i<8; i++) {
      join.data[1+i] = otaa.appeui[7-i];
    }

    for (uint8_t i=0; i<8; i++) {
      join.data[9+i] = otaa.deveui[7-i];
    }

    join.data[17] = (otaa.devnonce & 0x00FF);
    join.data[18] = ((otaa.devnonce >> 8) & 0x00FF);

    // uart_arr(WARN("pkg join"), join.data, join.len);

    join.len -= 4; // ignore mic
    uint8_t datam[4] = {0};
    Packet mic = { .data=datam, .len=4 };
    aes128_mic(otaa.appkey, &join, &mic); // len without mic
    join.len += 4;
    for (uint8_t i=0; i<4; i++) {
      join.data[join.len-4+i] = mic.data[i];
    }

    uint32_t tx_tick;
    rfm95.send(&join, session.frequencies[tx_frq_pos], tx_datarate, session.txpower, &tx_tick);
    uint32_t rx_window_tick = tx_tick + 5*TICKS_PER_SEC;

    DF("\n" BOLD("(%lu) %u. join request sent %lu SF%u") "\n", tx_tick, trials, session.frequencies[tx_frq_pos], tx_datarate);


    // rx1 window: 5sec + airtime(33, rx_datarate)
    // SF10 join accept ~5536ms (airtime: 452ms)
    if (rx_datarate < 12) {
      DF("(%lu) RX1: waiting for data %lu SF%u\n", rx_window_tick, session.frequencies[rx_frq_pos], rx_datarate);
      clock.sleep_until(rx_window_tick);
      if (rfm95.wait_for_single_package(session.frequencies[rx_frq_pos], rx_datarate) == OK && decode_join_accept() == OK) {
        session.txdatarate = tx_datarate;
        valid_lora = 1;
      }

      if (valid_lora) {
        DF(OK("(%lu) RX1 success") "\n", clock.current_tick);
        if (!wholescan) {
          _persist_session();
          return OK;
        }
      } else {
        DF(NOK("(%lu) RX1 timeout") "\n", clock.current_tick);
      }
    }

    // rx2 window: 6sec + airtime(33, rx_datarate)
    rx_datarate = 12;
    rx_frq_pos = 8; // 8695250
    valid_lora = 0;
    DF("(%lu) RX2: waiting for data %lu SF%u\n", rx_window_tick + TICKS_PER_SEC, session.frequencies[rx_frq_pos], rx_datarate);
    clock.sleep_until(rx_window_tick + TICKS_PER_SEC);
    if (rfm95.wait_for_single_package(session.frequencies[rx_frq_pos], rx_datarate) == OK && decode_join_accept() == OK) {
      session.txdatarate = tx_datarate;
      valid_lora = 1;
    }

    if (valid_lora) {
      DF(OK("(%lu) RX2 success") "\n", clock.current_tick);
      if (!wholescan) {
        _persist_session();
        return OK;
      }
    } else {
      DF(NOK("(%lu) RX2 timeout") "\n", clock.current_tick);
    }

    tx_datarate++;
    if (tx_datarate > 12) tx_datarate = 12;  // try SF7, SF8, ... until SF12

    trials++;
    if (trials < num_trials) {
      // DL(NOK("timeout!"));
      // TODO sleep 15sec, 30sec, 1min, 5min, 30min, 60min (repeat)
      // https://lora-developers.semtech.com/library/tech-papers-and-guides/the-book/joining-and-rejoining/
      uint8_t sleep_time = wholescan ? 5 : (uint8_t)(join.data[join.len-1]&0x07)*4+5; // pseudo random, taking 3 lsb from last byte of join, min 5sec
      DF("sleeping: %us\n", sleep_time);
      clock.sleep_for((uint32_t)sleep_time * TICKS_PER_SEC); // sleep random time 5-37sec
    }
  }

  return ERROR;
}

/*
 * returns position of frequency to use from session.frequencies
 * the useable frequencies are defined in session.chmask
 * chmask and frequencies are returned by the join_accept
 *
 */
uint8_t LORAWAN::get_next_frq_pos() {
  while (1) {
    chmask_current_pos += 1;
    if (chmask_current_pos == 16) chmask_current_pos = 0;
    if ((1<<chmask_current_pos) & session.chmask) return chmask_current_pos;
  }
}

Status LORAWAN::send(const Packet *payload, Packet *rx_payload, uint8_t ack) {
  return send(payload, rx_payload, 0, ack);
}

/*
 * datarate 7..12
 * datarate 0: use automatic datarate set by gw or join (from session)
 */
Status LORAWAN::send(const Packet *payload, Packet *rx_payload, const uint8_t datarate, uint8_t ack) {
  uint8_t len = payload->len+13;
  uint8_t tx_cmds[1] = {0};
  uint8_t tx_datarate = datarate ? datarate : session.txdatarate;

  ack = ack ? 1 : 0; // ensure only LSB is set
  // simple modulo, sending LinkCheckReq every 2^check_link_nth
  uint8_t do_check = check_link_nth && session.counter && ((session.counter>>check_link_nth)*(1<<check_link_nth) == session.counter);
  if (do_check) {
    queue_cmd_tx[queue_p] = LORA_MAC_LINKCHECK;
    queue_p++;
  }
  // TODO len += queue_p;
  uint8_t data[len];
  memset(data, 0, len);
  Packet lora = { .data=data, .len=len };
  create_package(payload, &lora, ack, 0, tx_cmds);

  uint8_t tx_frq_pos = get_next_frq_pos();

  PORTA.DIRSET = PIN7_bm;

  uint32_t tx_tick;
  rfm95.send(&lora, session.frequencies[tx_frq_pos], tx_datarate, session.txpower, &tx_tick);
  session.counter++;
  _persist_session();
  DF("(%lu) package sent %lu SF%u\n", tx_tick, session.frequencies[tx_frq_pos], tx_datarate);

  uint8_t rx_frq_pos = tx_frq_pos;
  uint8_t rx_datarate = tx_datarate + session.rxoffset;

  uint32_t rx_window_tick = tx_tick + session.rxdelay*TICKS_PER_SEC;

  // rx_frq_pos = 8;
  // rx_datarate = 9;
  /*
  rfm95.receive_continuous(session.frequencies[rx_frq_pos], rx_datarate);
  uint8_t f=0;
  while(1) {
    f = rfm95.read_reg(0x12);
    if (f&0x40) break;
  }
  DL(OK("RX success"));
  return decode_data_down(rx_payload, NULL, ack);
  */

  const uint8_t rx_cmds_len = 15;
  uint8_t rx_cmds_data[rx_cmds_len] = {0};
  Packet rx_cmds = { .data = rx_cmds_data, .len=rx_cmds_len };
  Status status_received;

  // only check for incoming data if rx_payload given
  if (rx_payload) {
    rx_payload->len = 0; // by default we get no data
    rx_cmds.len = 0; // by default no data
    // rx1 window
    // no rx1 window for SF12 as it would takte too much time for 1sec window (airtime 1.8sec)
    if (datarate != 12) {
      DF("(%lu) RX1: waiting for data %lu SF%u\n", rx_window_tick, session.frequencies[rx_frq_pos], rx_datarate);
      clock.sleep_until(rx_window_tick);
      PORTA.OUTCLR = PIN7_bm;
      if (rfm95.wait_for_single_package(session.frequencies[rx_frq_pos], rx_datarate) == OK) {
        // TODO if do_check and no LORA_MAC_LINKCHECK received -> rejoin
        // TODO handle ADR LinkADRReq, LinkADRAns (add to queue to send on next uplink)
        status_received = decode_data_down(rx_payload, &rx_cmds, ack);
        DF(OK("(%lu) RX1 success") "\n", clock.current_tick);
        if (rx_cmds.len) {
          process_cmds(&rx_cmds);
          uart_arr("rx cmds", rx_cmds.data, rx_cmds.len);
        }
        return status_received;
      }
    }

    // rx2 window
    rx_frq_pos = 8; // 8695250
    rx_datarate = session.rx2datarate;
    DF("(%lu) RX2: waiting for data %lu SF%u\n", rx_window_tick + TICKS_PER_SEC, session.frequencies[rx_frq_pos], rx_datarate);
    clock.sleep_until(rx_window_tick + TICKS_PER_SEC);
    if (rfm95.wait_for_single_package(session.frequencies[rx_frq_pos], rx_datarate) == OK) {
      status_received = decode_data_down(rx_payload, &rx_cmds, ack);
      DF(OK("(%lu) RX2 success") "\n", clock.current_tick);
      if (rx_cmds.len) {
        uart_arr("rx cmds", rx_cmds.data, rx_cmds.len);
      }
      return status_received;
    }

    return OK;
  } else {
    return OK;
  }
}

void LORAWAN::process_cmds(Packet *cmds) {
  uint8_t p=0; // TODO p and queue_p should be reset when sent
  queue_p = 0;
  uint8_t cmd;
  uint8_t session_has_changed = 0;

  while (p<cmds->len) {
    cmd = cmds->data[p];
    uint8_t response = 0;
    // datarate[7:4] txpower[3:0] | CHMask(2) | RFU[7] CHMaskCntl[6:4] NbTrans[3:0]
    if (cmd == LORA_MAC_LINKADR) {
      // TODO Nbtrans not yet implemented
      uint8_t datarate = cmds->data[p+1] >> 4;   // convert to SF7-12. DR0=SF12, DR5=SF7
      uint8_t txpower = cmds->data[p+1] & 0x0f;  // 0=max, 1=max-2db, ... 7=max-14db, max should be 16db, but RFM95: 23db-9db
      uint8_t redundancy = cmds->data[p+4];
      uint8_t chmaskcntl = (redundancy&~0x80)>>4;

      // accept chmask settings only if no bit except 1, 6 of chmaskcntl is set (regional parameters p.27 2.4.5. LinkAdrReq Command)
      if (!(chmaskcntl & ~((1<<0) | (1<<6)))) {
        if (chmaskcntl & (1<<6)) {
          session.chmask = 0xff;
        } else {
          session.chmask = (cmds->data[p+3]<<8) | cmds->data[p+2]; //bit 0: channel 1, bit 2: channel 2, ... bit 15: channel 16
        }
        response |= 0x01; // ChannelMaskACK
      }

      if (datarate != 0x0f && datarate <= 5) {
        session.txdatarate = 12 - datarate;     // convert to SF7-12. DR0=SF12, DR5=SF7
        response |= 0x02; // DataRateACK
      }

      if (txpower != 0x0f && txpower <=7) {
        session.txpower = 16 - 2*txpower;  // 0=max, 1=max-2db, ... 7=max-14db
        response |= 0x04; // PowerACK
      }
      queue_cmd_tx[queue_p++] = LORA_MAC_LINKADR;
      queue_cmd_tx[queue_p++] = response; // PowerACK | DataRateACK | ChannelMaskACK
      session_has_changed = response;
      p += 4; // data is 4 bits
    } else if (cmd == LORA_MAC_DUTYCYCLE) {
      // TODO not implemented so do not respond to it
      // queue_cmd_tx[queue_p] = LORA_MAC_DUTYCYCLE;
      // queue_p++;
      p += 1; //  data is 1 bit
    } else if (cmd == LORA_MAC_RXPARAM) {
      session.rxoffset = (cmds->data[p+1] & ~0x80)>>4;
      session.rx2datarate = 12 - (cmds->data[p+1] & 0x0f);
      // TODO frequency same as NEWCHANNEL command
      queue_cmd_tx[queue_p++] = LORA_MAC_RXPARAM;
      queue_cmd_tx[queue_p++] = 0x06; // RX1DROffsetACK ] RX2DataRateACK | ~ChannelACK
      session_has_changed = 1;
      p += 4;
    } else if (cmd == LORA_MAC_DEVSTATUS) {
      // TODO not implemented do not respond to it (needs to calculate and save SNR)
      p += 0; // no payload
    } else if (cmd == LORA_MAC_NEWCHANNEL) {
      queue_cmd_tx[queue_p++] = LORA_MAC_DEVSTATUS;
      queue_cmd_tx[queue_p++] = 0x00;
      session_has_changed = 1;
      p += 5;
    } else if (cmd == LORA_MAC_RXTIMING) {
      session.rxdelay = cmds->data[p+1] & 0x0f;
      queue_cmd_tx[queue_p++] = LORA_MAC_RXTIMING; // TODO should be added to upload data until download received from enddevice
      session_has_changed = 1;
      p += 1;
    } else if (cmd == LORA_MAC_TXPARAM) {
      p += 1;
      // TODO not implemented do not respond
    } else if (cmd == LORA_MAC_DICHANNEL) {
      // not implemented in EU863-870 -> drop it
      p += 4;
    } else if (cmd == LORA_MAC_DEVICETIME) {
      p +=5;
    }

    p +=1; // get next command
  }

  if (session_has_changed) {
    _persist_session();
  }
}

Status LORAWAN::decode_data_down(Packet *payload, Packet *rx_cmds, uint8_t ack_requested) {
  uint8_t datap[64] = {0};
  Packet phy = { .data=datap, .len=64 }; // PhyPayload MHDR(1) MAC Payload|Join Accept MIC(4)
  Status status = rfm95.read(&phy);
  // uart_arr("raw rx package", phy.data, phy.len);

  // check crc
  if (status != OK) {
    DF("crc error (%u)\n", status);
    return status;
  }

  // check data type (mhdr)
  uint8_t mhdr = phy.data[0];
  if (mhdr != 0x60 && mhdr != 0xa0) {
    DF(NOK("unsupported type:") "%02x\n", mhdr);
    return ERROR;
  }

  // check devaddr
  uint8_t len = 4;
  uint8_t data[len];
  memset(data, 0, len);
  Packet mic = { .data=data, .len=len };

  uint8_t devaddr[4] = {0};
  for (uint8_t i=0; i<4; i++) {
    devaddr[i] = phy.data[4-i]; // phy.data[4..1]
  }
  // uart_arr("data for", devaddr_received, 4);
  if (!is_same(session.devaddr, devaddr, 4)) {
    DL(NOK("devaddr error"));
    uart_arr("  got     : ", devaddr, 4);
    uart_arr("  expected: ", session.devaddr, 4);
    return ERROR;
  }

  // uart_arr("you've got mail", phy.data, phy.len);

  // check mic by calculating mic over received msg (see https://tools.ietf.org/html/rfc4493 chapter 2.5)
  phy.len -= 4; // ignore mic for calculation
  uint16_t counter = phy.data[7];
  counter = (counter << 8) + phy.data[6];
  uint8_t direction = 1;

  uint8_t lenb0 = phy.len+16;
  uint8_t datab0[lenb0];
  memset(datab0, 0, lenb0);
  Packet b0 = { .data=datab0, .len=lenb0 };
  aes128_b0(&phy, counter, direction, devaddr, &b0);
  aes128_mic(session.nwkskey, &b0, &mic);
  if (!is_same(mic.data, &phy.data[phy.len], 4)) {
    DL(NOK("MIC fail!"));
    return ERROR;
  }

  // get frmpayload
  // TODO if confirmed, ack must be set in header on the next uplink
  // mhdr(1) devaddr(4) fctrl(1) fcnt(2)
  // fctrl = phy.data[5] -> foptslen phy.data[5] bits 3..0
  // counter = phy.data[6..7]

  uint8_t fctrl = phy.data[5];
  // DF("fctrl: 0x%02x\n", fctrl);
  uint8_t frame_options_len = (fctrl & 0x07); // FCtrl[3..0] if set, we have some options (=mac cmds)
  uint8_t ack_received = ((fctrl>>5) & 0x01); // FCtrl[5] if ack was requested while sending, this ack should be set in the response
  // DF("foptslen: %u\n", frame_options_len);
  DF("ack received: %u\n", ack_received);
  DF("adr received: %u\n", ((fctrl>>7) & 0x01)); // FCtrl[7]
  // we got mac commands
  if (frame_options_len) {
    rx_cmds->len = frame_options_len;
    for (uint8_t i=0; i<frame_options_len; i++) {
      rx_cmds->data[i] = phy.data[8+i];
    }
  }
  uint8_t fport = phy.data[8+frame_options_len];
  uint8_t pdata = 9 + frame_options_len;            // MHDR(1) + FHDR(7+FOptsLen) + FPort(1)

  // downlink data
  if (pdata < phy.len) {
    payload->len = phy.len - pdata;
    for (uint8_t i=0; i<payload->len; i++) {
      payload->data[i] = phy.data[pdata+i];
    }
    // decrypt payload_encrypted by encrypting it (=FRMPayload)
    cipher(payload, counter, direction, fport, devaddr);
    return (ack_requested && ack_received) || !ack_requested ? OK : NO_ACK;
  // mac commands available
  } else {
    return (ack_requested && ack_received) || !ack_requested ? OK : NO_ACK;
  }
}

/*
 * TODO LinkADRReq mac cmd sets the channels
 */
Status LORAWAN::decode_join_accept() {
  uint8_t datap[64] = {0};
  Packet phy = { .data=datap, .len=64 }; // PhyPayload MHDR(1) MAC Payload|Join Accept MIC(4)

  // check crc
  Status status = rfm95.read(&phy);
  // uart_arr("raw join accept package", phy.data, phy.len);
  if (status != OK) {
    DF("error (%u)\n", status);
    return status;
  }

  // check data type (mhdr)
  uint8_t mhdr = phy.data[0];
  if (mhdr != 0x20) {
    DF(NOK("unsupported type:") "%02x\n", mhdr);
    return ERROR;
  }

  // decrypt
  // mhdr is not part of decrypting
  // -> loop over len-1 and +1 in aes128_encrypt
  for (uint8_t i=0; i<((phy.len-1)/16); i++) {
    aes128_encrypt(otaa.appkey, &(phy.data[i*16+1]));
  }
  // uart_arr("phypayload decrypted", phy.data, phy.len);
  // calculate mic and compare with received mic
  // mhdr is part of mic calculation
  phy.len -= 4; // "remove" mic

  uint8_t len = 4;
  uint8_t data[len];
  memset(data, 0, len);
  Packet mic = { .data=data, .len=len };
  aes128_mic(otaa.appkey, &phy, &mic);
  if (!is_same(mic.data, &phy.data[phy.len], 4)) {
    DL(NOK("MIC fail!"));
    return ERROR;
  }

  // appnonce   = phy.data[1..3]
  // netid      = phy.data[4..6]
  // devaddr    = phy.data[10..7]
  // dlsettings = phy.data[11]
  // rxdelay    = phy.data[12]
  // cflist     = phy.data[13..29]
  for (uint8_t i=0; i<16; i++) session.appskey[i] = 0x00; // be sure to clear, ie also if we already have a key from earlier joins
  session.appskey[0] = 0x02;
  for(uint8_t i=1; i<7; i++) { // apnonce(3) netid(3)
    session.appskey[i] = phy.data[i];
  }
  session.appskey[7] = otaa.devnonce & 0x00ff;
  session.appskey[8] = (otaa.devnonce >> 8) & 0x00ff;
  aes128_copy_array(session.nwkskey, session.appskey);
  session.nwkskey[0] = 0x01;
  aes128_encrypt(otaa.appkey, session.appskey);
  aes128_encrypt(otaa.appkey, session.nwkskey);

  for (uint8_t i=0; i<4; i++) {
    session.devaddr[i] = phy.data[10-i];
  }

  session.rxdelay = phy.data[12];
  session.rxoffset = (phy.data[11]>>4) & 0x07; // dlsettings[6..4]
  session.rx2datarate = 12 - (phy.data[11] & 0x07);   // dlsettings[3..0] returns DRx, DR0=SF12, DR6=SF07
  // DF("dlsettings: 0x%02x\n", phy.data[11]);
  // DF("rxdelay: %u\n", session.rxdelay);
  // DF("rxoffset: %u\n", session.rxoffset);
  // DF("rx2datarate: %u\n", session.rx2datarate);

  // uart_arr("appskey", session.appskey, 16);
  // uart_arr("nwkskey", session.nwkskey, 16);
  // uart_arr("devaddr", session.devaddr, 4);

  // cflist = list of frequencies
  DF("len: %u\n", phy.len);
  if (phy.len == 29 && phy.data[28] == 0) { // CFListType = 0 -> a list of 5 add frequencies is available
    for (uint8_t i=0; i<5; i++) {
       session.frequencies[3+i] = 0;
      for (uint8_t ii=0; ii<3; ii++) {
        session.frequencies[3+i] = (session.frequencies[3+i]<<8) + phy.data[13+(2-ii)+3*i];
      }
      // DF("frq ch%u: %lu\n", 3+i, session.frequencies[3+i]);
    }
    session.chmask = 0xff;
  }
  return OK;
}

/*
 * according to https://hackmd.io/@hVCY-lCeTGeM0rEcouirxQ/S1kg6Ymo-?type=view#MAC-Frame-Payload-Encryption
 */
void LORAWAN::cipher(Packet *payload, const uint16_t counter, const uint8_t direction, const uint8_t fport, const uint8_t *devaddr) {
  uint8_t block_a[16];
  uint8_t number_of_blocks = payload->len / 16;

  for (uint8_t block_count=0; block_count<=number_of_blocks; block_count++) {
    // create block A_i
    memset(block_a, 0, sizeof(block_a));
    block_a[0] = 0x01;
    block_a[5] = direction; // 0=send, 1=receive
    for (uint8_t i=0; i<4; i++) block_a[6+i] = (devaddr ? devaddr : session.devaddr)[3-i];
    block_a[10] = (counter & 0x00ff);
    block_a[11] = ((counter >> 8) & 0x00ff);
    block_a[15] = block_count+1;  // we iterate from 0 instead of 1 as in the spec

    aes128_encrypt(fport ? session.appskey : session.nwkskey, block_a); // usually APPSKEY

    // only xor if data available, no padding with 0x00!
    uint8_t remaining = payload->len - 16*block_count;
    for (uint8_t i=0; i<((remaining > 16) ? 16 : remaining); i++) {
      payload->data[16*block_count+i] = payload->data[16*block_count+i] ^ block_a[i];
    }
  }
}

/*
 * lorawan packet: header+port(9) + FOpts(0..15) + FRMpayload() + mic(4)
 * payload: len
 * lora: len+13+15 (fopts worst case)
 * (with B0: len+9+16)
 *
 * package:     40 11 34 01 26 00 05 00 01 6f c9 03
 * b0: 49 00 00 00 00 00 11 34 01 26 05 00 00 00 00 0c 40 11 34 01 26 00 05 00 01 6f c9 03
 * mic: 76 0c e1 ab
 * package+mic: 40 11 34 01 26 00 05 00 01 6f c9 03 76 0c e1 ab
 *
 * cipher: 61 62 63 -> 6f c9 03
 *
 * TODO send mac cmds as data frame with fport=0 (encrypted)
 * TODO respond to ack requests from gw (downlink)
 * TODO process queue_cmd_tx
 *
 */
void LORAWAN::create_package(const Packet *payload, Packet *lora, uint8_t ack, uint8_t adaptive_ack, uint8_t *cmds) {
  uint8_t cmds_count = lora->len - payload->len - 13;
  if (cmds_count > 15) cmds_count = 15;
  // DF("cmds_count: %u\n", cmds_count);
  if (cmds_count) uart_arr("tx mac", cmds, cmds_count);
  uint8_t direction = 0;

  // header
  lora->data[0] = (0x40<<ack);                                          // mhdr(1), data uplink (unconfirmed: 0x40, confirmed: 0x80)
  for (uint8_t i=0; i<4; i++) lora->data[1+i] = session.devaddr[3-i];   // devaddr(4)
  lora->data[5] = (adaptive<<7) | (adaptive_ack<<6) | cmds_count;       // fctrl(1) adr[7] adrackreq[6] ack[5] rfu[4] foptslen[3..0]
  // lora->data[5] = (adaptive<<7) | cmds_count;                        // fctrl(1) adr[7] adrackreq[6] ack[5] rfu[4] foptslen[3..0]
  lora->data[6] = (session.counter & 0x00ff);                           // fcnt(1) tx counter lsb
  lora->data[7] = ((session.counter >> 8) & 0x00ff);                    // fcnt(1) tx counter msb
  for (uint8_t i=0; i<cmds_count; i++) {                                // fopts(0..15) mac commands
    lora->data[8+i] = cmds[i];
  }
  lora->data[8+cmds_count] = 0x01;                                      // fport(0..1) hardcoded for now. port>1 use AppSKey else NwkSkey
                                                                        // frmpayload(N)

  // encrypt payload (=FRMPayload)
  uint8_t data[payload->len];
  memset(data, 0, payload->len);
  Packet payload_encrypted = { .data=data, .len=payload->len };
  payload_encrypted.len = payload->len;
  for (uint8_t i=0; i<payload_encrypted.len; i++) {
    payload_encrypted.data[i] = payload->data[i];
  }
  cipher(&payload_encrypted, session.counter, direction, lora->data[8+cmds_count]);
  for (uint8_t i=0; i<payload_encrypted.len; i++) {
    lora->data[9+cmds_count+i] = payload_encrypted.data[i];
  }
  lora->len -= 4; // ignore mic for now

  // calculate mic of package consisting of header + data
  uint8_t lenb0 = payload->len+9+cmds_count+16;
  uint8_t datab0[lenb0];
  memset(datab0, 0, lenb0);
  Packet b0 = { .data=datab0, .len=lenb0 };
  aes128_b0(lora, session.counter, direction, session.devaddr, &b0);
  uint8_t dmic[4] = {0};
  Packet mic = { .data=dmic, .len=4 };
  aes128_mic(session.nwkskey, &b0, &mic);

  // load calculated mic
  lora->len += 4;
  for (uint8_t i=0; i<4; i++) {
    lora->data[lora->len-4+i] = mic.data[i];
  }
  // uart_arr("lora", lora->data, lora->len);
}

uint8_t LORAWAN::is_same(uint8_t *arr1, uint8_t *arr2, uint8_t len) {
  for (uint8_t i=0; i<len; i++) {
    if (arr1[i] != arr2[i]) return 0;
  }
  return 1;
}

/*
 *
 * https://docs.google.com/spreadsheets/d/19SEkw9gAO1NuBZgpJsPshfnaFuJ7NUc4utmLDjeL3eo/edit#gid=585312605
 *
 * len: total length of packet to send
 * datarate: 7..12 for SFx
 *
 * returns airtime in ms
 */
uint16_t LORAWAN::calc_airtime(uint8_t len, uint8_t datarate) {
  uint16_t bandwith = 125;
  uint8_t preamble = 8;
  uint8_t explicit_header = 1;
  uint8_t coding_rate = 5;
  uint8_t de = 0;

  de = datarate > 10 ? 1:0;
  uint32_t tsym = ((uint32_t)128<<datarate)/bandwith;           // 128 for better precision
  uint16_t tpreamble = (preamble*128 + 4.25*128)*tsym/128;      // 128 for better precision
  int32_t temp = (16*8*len-4*datarate+28-16-20*(1-explicit_header)) / (4*(datarate-2*de)); // *16 for better precision
  if (temp<0) temp = 0;
  uint8_t symbnb = 8+(temp+16)/16*coding_rate;
  uint32_t tpaypload = symbnb*tsym;

  return (tpreamble+tpaypload)/128;
}
