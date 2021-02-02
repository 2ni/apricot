#include <avr/io.h>
#include <util/delay.h>
#include "string.h"

#include "lorawan.h"
#include "lorawan_struct.h"
#include "aes.h"
#include "cmac.h"
#include "uart.h"
#include "rfm95.h"
#include "sleep.h"
#include "timer.h"
#include "millis.h"

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
 * when adr these defaults are set:
 * - max allowed tx power
 * - data rate from join or minimum (SF12)
 *
 * if datarate > proposed datarate || txpower < proposed txpower or to check connectivity:
 * set ADRACKReq after n transmissiosn -> GW must respond. If received -> clear ADRACKReq
 * if not received -> set txpower to default, then switch to next lower data rate
 * at the end rejoin
 */
LORAWAN::LORAWAN() {
  LORAWAN(1, 0);
}

LORAWAN::LORAWAN(uint8_t iadr, uint8_t i_check_link_nth) {
  check_link_nth = i_check_link_nth;
  adr = iadr ? 1 : 0;
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

/*
 * to avoid tests blocking, this needs to be in a separate function
 * which must be called in the real programm but not in the test suite
 */
// void LORAWAN::init(pins_t *cs, pins_t *dio0, pins_t *dio1) {
void LORAWAN::init(pins_t *cs) {
  session.counter = 10;
  session.rxdelay = 0;
  session.rxoffset = 0;
  session.rx2datarate = 9;
  session.txpower = 16;

  /*
  uint8_t f[16] = { 0xF0, 0x99, 0x27, 0xA6, 0x77, 0x4E, 0x47, 0x35, 0x5D, 0xDE, 0x94, 0x43, 0xC5, 0x15, 0x17, 0xAB };
  for (uint8_t i=0; i<16; i++) session.nwkskey[i] = f[i];
  uint8_t g[16] = { 0xBC, 0x31, 0x2D, 0x42, 0xEF, 0x25, 0x5B, 0x0A, 0x2A, 0xD2, 0xAC, 0xFE, 0xED, 0x88, 0x5E, 0x66 };
  for (uint8_t i=0; i<16; i++) session.appskey[i] = g[i];
  uint8_t h[4] = {0x26, 0x01, 0x42, 0xd1};
  for (uint8_t i=0; i<4; i++) session.devaddr[i] = h[i];
  */

  // RFM95 rfm95(cs, dio0, dio1);
  RFM95 rfm95(cs);
  uint8_t version = rfm95.init();
  if (!version) {
    DL(NOK("RFM95 init failed. Stopped."));
    while (1);
  } else {
    DF("RFM95 version: 0x%02x\n", version);
  }
}

/*
 * https://lora-alliance.org/sites/default/files/2020-06/rp_2-1.0.1.pdf
 * RECEIVE_DELAY1 1s
 * RECEIVE_DELAY2 2s (RECEIVE_DELAY1 + 1s)
 *
 * JOIN_ACCEPT_DELAY1 5sec
 * JOIN_ACCEPT_DELAY2 6sec (869.525MHz, SF12)
 *
 * join accept: 33 bytes -> airtime: 1.8sec
 * -> 6sec + 1.8sec = 7.8sec
 *
 * according to @Pshemek:
 *  join accept for SF7 is sent 4s after join request, for SF12 after exactly 5s + airtime
 *
 * SF7 and SF8 should have a join reply in RX1 using the parameters of the request if gw has available airtime
 * SF9+ response is in RX2 at SF12
 *
 * what works
 * - join request: ch 4, SF9,  join accept: ch 4, SF9
 * - join request: ch 7, SF9,  join accept: ch 7,  SF9  (~5.3sec)
 * - join request: ch 7, SF10, join accept: ch 99, SF12 (~6.8sec)
 * - join request: ch 6, SF11, join accept: ch 99, SF12 (~7.8sec)
 * - join request: ch x, SF12, join accept: ch 99, SF12 (~6.8-8.0sec)
 *
 */

// TIMER timer;

/*
 * return 0: no errors
 *
 * tries to join several time by lowering the datarate each time
 * maybe it's just better to just start at SF12, as we rarely get any response before
 *
 * CAUTION: we have an additional waiting of 50ms because millis_time() seems to be ~50ms too fast
 *
 * TODO set attiny to sleep and wake up on pin interrupt rfm_interrupt or time_out
 *
 * TODO save received cflist, offset and delay
 * TODO handle sending power (rssi)
 *
 */
Status LORAWAN::join(uint8_t wholescan) {
  uint8_t tx_channel = 0;
  uint8_t tx_datarate = 7;

  uint8_t rx_channel;
  uint8_t rx_datarate;

  uint8_t valid_lora;
  uint8_t trials = 0;
  uint32_t now;
  uint16_t airtime = 0;
  uint16_t num_trials = 7;

  // we depend on timings so we init it, if not yet done
  if (!millis_is_init()) {
    millis_init();
  }

  // we try multiple times to get a join accept package (trying to fullfill retransmission on p.65 of 1.0.3 specification)
  while (trials < num_trials) {
    valid_lora = 0;
    otaa.devnonce = rfm95.get_random();
    // tx_channel = (uint8_t)(rfm95.get_random(8)) / 64 + 4;             // random channel 4+0-3
    tx_channel = (tx_channel+1) % 3; // iterate through channel 0..2
    rx_channel = tx_channel;
    rx_datarate = tx_datarate;
    uint16_t sleep_window_rx1= 4990;

    // use correct datarates, channels for join request and accept listening
    // see "receive windows" in https://lora-alliance.org/sites/default/files/2018-05/lorawan_regional_parameters_v1.0.2_final_1944_1.pdf
    // (RX1DROffset: 0)
    // join accept SF7-SF11 -> RX1 after 5sec: SF7-SF12
    // join accept SF7-SF12 -> RX2 after 6sec: SF12, 869.525MHz

    send_join_request(tx_channel, tx_datarate);
    now = millis_time();
    DF("\n" BOLD("%u. join request sent ch%u SF%u") "\n", trials, tx_channel, tx_datarate);


    // rx1 window: 5sec + airtime(33, rx_datarate)
    // SF10 join accept ~5536ms (airtime: 452ms)
    if (rx_datarate < 12) {
      airtime = calc_airtime(33, rx_datarate); // join request len = 33bytes
      // timer.stop();
      DF("(%lu) RX1: waiting for data ch%u SF%u (air: %u)\n", millis_time()-now, rx_channel, rx_datarate, airtime);
      // while ((millis_time()-now) < (5050)) {}
      sleep_ms(sleep_window_rx1-(millis_time()-now));
      now = millis_time();
      if (rfm95.wait_for_single_package(rx_channel, rx_datarate) == OK && decode_join_accept() == OK) {
        session.txdatarate = tx_datarate;
        valid_lora = 1;
      }
      /*
      rfm95.receive_continuous(rx_channel, rx_datarate);
      timer.start(100+airtime + airtime/8);

       while (!timer.timed_out() && !valid_lora) {
        // rfm_interrupt==1 -> we have data ready
        if (get_output(&rfm_interrupt) && decode_join_accept() == OK) {
          valid_lora = 1;
        }
      }
      */

      if (valid_lora) {
        DF(OK("(%lu) RX1 success") "\n", millis_time()-now);
        if (!wholescan) {
          return OK;
        }
      } else {
        DF(NOK("(%lu) RX1 timeout") "\n", millis_time()-now);
      }
      sleep_window_rx1 = 0;
    }

    // rx2 window: 6sec + airtime(33, rx_datarate)
    // receive_continuous needs to start at 6000ms
    // join accept ~7946ms (airtime: 1810ms)
    // receive time in middle of preamble except time
    rx_datarate = 12;
    rx_channel = 99;
    valid_lora = 0;
    airtime = calc_airtime(33, rx_datarate);
    // timer.stop();
    DF("(%lu) RX2: waiting for data ch%u SF%u (air: %u)\n", millis_time()-now, rx_channel, rx_datarate, airtime);
    // while ((millis_time()-now) < (6060)) {}
    sleep_ms(sleep_window_rx1+990-(millis_time()-now));
    if (rfm95.wait_for_single_package(rx_channel, rx_datarate) == OK && decode_join_accept() == OK) {
      session.txdatarate = tx_datarate;
      valid_lora = 1;
    }
    /*
    rfm95.receive_continuous(rx_channel, rx_datarate);
    timer.start(100+airtime + airtime/8);

    while (!timer.timed_out() && !valid_lora) {
      // rfm_interrupt==1 -> we have data ready
      if (get_output(&rfm_interrupt) && !decode_join_accept() = OK) {
        valid_lora = 1;
      }
    }
    */

    if (valid_lora) {
      DF(OK("(%lu) RX2 success") "\n", millis_time()-now);
      if (!wholescan) {
        return OK;
      }
    } else {
      DF(NOK("(%lu) RX2 timeout") "\n", millis_time()-now);
    }

    tx_datarate++;
    if (tx_datarate > 12) tx_datarate = 12;  // try SF7, SF8, ... until SF12

    trials++;
    if (trials < num_trials) {
      // DL(NOK("timeout!"));
      // TODO sleep 15sec, 30sec, 1min, 5min, 30min, 60min (repeat)
      // https://lora-developers.semtech.com/library/tech-papers-and-guides/the-book/joining-and-rejoining/
      uint8_t sleep_time = wholescan ? 5 : (uint8_t)(5+rfm95.get_random(8) / 8);
      DF("sleeping: %us\n", sleep_time);
      sleep_s(sleep_time); // sleep random time 5-37sec
    }
  }

  // timer.stop();

  /* we get appskey, nwkskey, devaddr
  if (valid_lora) {
    uart_arr("appskey", appskey, 16);
    uart_arr("nwkskey", nwkskey, 16);
    uart_arr("devaddr", devaddr, 4);
  }
  */

  return ERROR;
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

  uint8_t channel = (uint8_t)(rfm95.get_random(2)); // random channel 0-3

  // we depend on timings so we init it, if not yet done
  if (!millis_is_init()) {
    millis_init();
  }

  rfm95.send(&lora, channel, tx_datarate);
  uint32_t now = millis_time();
  session.counter++;
  DF("package sent ch%u SF%u\n", channel, tx_datarate);

  uint8_t rx_channel = channel;
  uint8_t rx_datarate = tx_datarate + session.rxoffset;

  uint16_t sleep_window_rx1 = session.rxdelay*1000-10; // convert in ms, for 1s -> 990ms

  /*
  rx_channel = 99;
  rx_datarate = 9;
  uint8_t valid_lora = 0;
  rfm95.receive_continuous(rx_channel, rx_datarate);
  while ((millis_time() - now) < 3000 && !valid_lora) {
    if (get_output(&rfm_interrupt) && decode_data_down(rx_payload) == OK) {
      valid_lora = 1;
    }
  }
  if (valid_lora) {
    DF(OK("(%lu) RX success") "\n", millis_time()-now);
    return OK;
  } else {
    DF(NOK("(%lu) RX timeout") "\n", millis_time()-now);
    return ERROR;
  }
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
      DF("(%lu) RX1: waiting for data ch%u SF%u\n", millis_time()-now, rx_channel, rx_datarate);
      // while ((millis_time()-now) < 1010);
      sleep_ms(sleep_window_rx1-(millis_time()-now));
      now = millis_time();
      if (rfm95.wait_for_single_package(rx_channel, rx_datarate) == OK) {
        DL(OK("received rx1"));
        // TODO if do_check and no LORA_MAC_LINKCHECK received -> rejoin
        // TODO handle adr LinkADRReq, LinkADRAns (add to queue to send on next uplink)
        status_received = decode_data_down(rx_payload, &rx_cmds, ack);
        if (rx_cmds.len) {
          process_cmds(&rx_cmds);
          uart_arr("rx cmds", rx_cmds.data, rx_cmds.len);
        }
        return status_received;
      }
      sleep_window_rx1 = 0;
    }

    // rx2 window
    rx_channel = 99;
    rx_datarate = session.rx2datarate;
    DF("(%lu) RX2: waiting for data ch%u SF%u\n", millis_time()-now, rx_channel, rx_datarate);
    // while ((millis_time()-now) < 2020);
    sleep_ms(sleep_window_rx1+990-(millis_time()-now));
    if (rfm95.wait_for_single_package(rx_channel, rx_datarate) == OK) {
      DL(OK("received rx2"));
      status_received = decode_data_down(rx_payload, &rx_cmds, ack);
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
  uint8_t p=0;
  uint8_t cmd;

  while (p<cmds->len) {
    cmd = cmds->data[p];
    if (cmd == LORA_MAC_LINKADR) {
      uint8_t datarate = cmds->data[p+1] >> 4;   // convert to SF7-12. DR0=SF12, DR5=SF7
      uint8_t txpower = cmds->data[p+1] & 0x0f;  // 0=max, 1=max-2db, ... 7=max-14db, max should be 16db, but RFM95: 23db-9db
      // TODO ChMask, ChaskCntl, Nbtrans not implemented yet

      if (datarate != 0x0f) {
        session.txdatarate = 12 - datarate;     // convert to SF7-12. DR0=SF12, DR5=SF7
      }

      if (txpower != 0x0f) {
        session.txpower = 16 - 2*txpower;  // 0=max, 1=max-2db, ... 7=max-14db
      }
      queue_cmd_tx[queue_p] = LORA_MAC_LINKADR;
      queue_p++;
      queue_cmd_tx[queue_p] = 0x06; // PowerACK | DataRateACK (we don't ack channel, as not yet implemented)
      queue_p++;
      p += 4;
    } else if (cmd == LORA_MAC_CYCLE) {
    } else if (cmd == LORA_MAC_RXPARAM) {
    } else if (cmd == LORA_MAC_DEVSTATUS) {
    } else if (cmd == LORA_MAC_NEWCHANNEL) {
    } else if (cmd == LORA_MAC_RXTIMING) {
    } else if (cmd == LORA_MAC_TXPARAM) {
    } else if (cmd == LORA_MAC_DICHANNEL) {
    }
  }
}

Status LORAWAN::decode_data_down(Packet *payload, Packet *rx_cmds, uint8_t ack_requested) {
  uint8_t datap[64] = {0};
  Packet phy = { .data=datap, .len=64 }; // PhyPayload MHDR(1) MAC Payload|Join Accept MIC(4)
  Status status = rfm95.read(&phy);
  // uart_arr("raw rx package", phy.data, phy.len);

  // check crc
  if (status != OK) {
    DF("error (%u)\n", status);
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
  uint8_t adr = ((fctrl>>7) & 0x01);               // FCtrl[7]
  // DF("foptslen: %u\n", frame_options_len);
  DF("ack received: %u\n", ack_received);
  DF("adr received: %u\n", adr);
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
 * TODO save frequencies for channel 3-7 on join accept message
 * uint8_t[8] frequencies;
 * uint8_t[8] channels; (8 is max length)
 *
 * TODO configure which channels can be used in the session
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
  session.counter = 0;
  // DF("dlsettings: 0x%02x\n", phy.data[11]);
  // DF("rxdelay: %u\n", session.rxdelay);
  // DF("rxoffset: %u\n", session.rxoffset);
  // DF("rx2datarate: %u\n", session.rx2datarate);

  // cflist = list of frequencies
  // uart_arr("appskey", session.appskey, 16);
  // uart_arr("nwkskey", session.nwkskey, 16);
  // uart_arr("devaddr", session.devaddr, 4);
  /*
  // frq
  uint32_t frq;
  for (uint8_t i=0; i<5; i++) {
    frq = 0;
    for (uint8_t ii=0; ii<3; ii++) {
      frq = (frq<<8) + msg[13+(2-ii)+3*i];
    }
    DF("frq ch%u: %lu\n", i+4, frq);
  }
  */
  return OK;
}

void LORAWAN::send_join_request(uint8_t channel, uint8_t datarate) {
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

  rfm95.send(&join, channel, datarate);
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
void LORAWAN::create_package(const Packet *payload, Packet *lora, uint8_t ack, uint8_t adrack, uint8_t *cmds) {
  uint8_t cmds_count = lora->len - payload->len - 13;
  if (cmds_count > 15) cmds_count = 15;
  // DF("cmds_count: %u\n", cmds_count);
  if (cmds_count) uart_arr("tx mac", cmds, cmds_count);
  uint8_t direction = 0;

  // header
  lora->data[0] = (0x40<<ack);                                          // mhdr(1), data uplink (unconfirmed: 0x40, confirmed: 0x80)
  for (uint8_t i=0; i<4; i++) lora->data[1+i] = session.devaddr[3-i];   // devaddr(4)
  lora->data[5] = (adrack<<6) | (adr<<7) | cmds_count;                  // fctrl(1) adr[7] adrackreq[6] ack[5] rfu[4] foptslen[3..0]
  // lora->data[5] = (adr<<7) | cmds_count;                                // fctrl(1) adr[7] adrackreq[6] ack[5] rfu[4] foptslen[3..0]
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
  uart_arr("lora", lora->data, lora->len);
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
