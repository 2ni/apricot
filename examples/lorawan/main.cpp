#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#include "uart.h"
#include "pins.h"
#include "mcu.h"
#include "lorawan.h"
#include "keys.h"
#include "touch.h"
#include "sht20.h"
#include "isl29035.h"
#include "mcu.h"

#define OTAA

// to avoid complex divisions and minimize the wakeup time
// we calculate the needed timer count beforehand
// and use the internal function _s_sleep()
#define SLEEP_TIME 50
#define SLEEP_PER UINT32_C(SLEEP_TIME)*1024/1000

LORAWAN lora;
SHT20 sht20;
ISL29035 isl;

uint8_t is_seated = 0;
uint32_t start_tick = 0;

uint8_t  EEMEM ee_have_session;
Lora_session EEMEM ee_session;
uint8_t have_session;

void print_session() {
  uart_arr("nwkskey", lora.session.nwkskey, 16);
  uart_arr("appskey", lora.session.appskey, 16);
  uart_arr("devaddr", lora.session.devaddr, 4);
  DF("counter      : %u\n", lora.session.counter);
  DF("tx datarate  : %u\n", lora.session.txdatarate);
  DF("rx delay     : %u\n", lora.session.rxdelay);
  DF("rx offset    : %u\n", lora.session.rxoffset);
  DF("rx2 datarate : %u\n", lora.session.rx2datarate);
  DF("tx power     : %u\n", lora.session.txpower);
  DF("chmask       : 0x%02x\n", lora.session.chmask);
  D("frequencies  : ");
  for (uint8_t i=0; i<8; i++) {
    if ((lora.session.chmask>>i) & 0x01) {
      DF("%lu, ", lora.session.frequencies[i]);
    }
  }
  DL("");
}

void join() {
  extern uint8_t DEVEUI[8];
  extern uint8_t APPEUI[8];
  extern uint8_t APPKEY[16];
  lora.set_otaa(DEVEUI, APPEUI, APPKEY);
  eeprom_update_byte(&ee_have_session, 0); // delete session

  if (lora.join() == OK) {
    have_session = 1;
    eeprom_update_byte(&ee_have_session, 23);
    eeprom_update_block(&lora.session, &ee_session, sizeof(lora.session));
    print_session();
  } else {
    DL("join failed");
    while (1);
  }
}

void send() {
  // hsb lsb - temp, hum, lux
  uint8_t len = 8;
  uint8_t data[len] = { 0 };
  Packet payload = { .data=data, .len=len };

  uint8_t rx_len = 64;
  uint8_t rx_data[rx_len] = {0};
  Packet rx_packet = { .data=rx_data, .len=rx_len };

  DF("counter      : %u\n", lora.session.counter);

  uint16_t temp = sht20.get_temperature();
  uint16_t hum = sht20.get_humidity();
  uint16_t lux = 0;
  isl.read_visible_lux(&lux);
  uint16_t vin = get_vin();

  payload.data[0] = (temp >> 8) & 0xff;
  payload.data[1] = temp & 0xff;
  payload.data[2] = (hum >> 8) & 0xff;
  payload.data[3] = hum & 0xff;
  payload.data[4] = (lux >> 8) & 0xff;
  payload.data[5] = lux & 0xff;
  payload.data[6] = (vin >> 8) & 0xff;
  payload.data[7] = vin & 0xff;
  Status status = lora.send(&payload, &rx_packet);
  eeprom_update_word(&ee_session.counter, lora.session.counter);
  if (status == OK) {
    DL(OK("all good"));
  } else if (status == NO_ACK) {
    DL(NOK("ack failed"));
  }

  if (rx_packet.len) {
    uart_arr("received message", rx_packet.data, rx_packet.len);
  } else {
    DL("no data received");
  }
}

void callback_button(TOUCH::Press_type type, uint32_t ticks) {
  switch (type) {
    case TOUCH::SHORT:
      send();
      break;
    case TOUCH::LONG:
      lora.session.txdatarate += 1;
      if (lora.session.txdatarate > 12) lora.session.txdatarate = 7;
      DF("datarate: %u\n", lora.session.txdatarate);
      break;
    default:
      break;
  }
}

int main(void) {
  mcu_init();
  sht20.init();
  if (isl.init() != 0) {
    DL(NOK("ISL29035 init failed"));
  }
  isl.set_ranges(1, 1); // 4000Lux, 12bit

  lora.init();

  TOUCH button(&PB7);

  uint8_t have_session = eeprom_read_byte(&ee_have_session) == 23 ? 1 : 0;
  if (have_session) {
    DL("load session from eeprom");
    eeprom_read_block(&lora.session, &ee_session, sizeof(lora.session));
    print_session();
  }

#ifdef OTAA
  uint32_t last_join_tick = 0;
  uint32_t last_data_tick = 0;
  uint8_t enforce = 0;

  if (!have_session) {
    join();
    last_join_tick = clock.current_tick;
    enforce = 1;
  }

  while (1) {
    button.is_pressed(&callback_button);

    // 120*60*1000*32768/8000 = 2h
    if (clock.current_tick >= (last_join_tick + 29491200)) {
      last_join_tick = clock.current_tick;
      join();
      enforce = 1;
    }

    // 10*60*1000*32768/8000 = 10min
    if (enforce || clock.current_tick >= (last_data_tick + 2457600)) {
      last_data_tick = clock.current_tick;
      send();
      enforce = 0;
    }

    // 50ms*32768/1000/8 = 204.8
    clock.sleep_for(205);
  }
#else
  lora.session.chmask = 0xff;
  lora.session.frequencies[3] = 8671000;
  lora.session.frequencies[4] = 8673000;
  lora.session.frequencies[5] = 8675000;
  lora.session.frequencies[6] = 8677000;
  lora.session.frequencies[7] = 8679000;
  DF("session: %u\n", have_session);
  if (!have_session) {
    extern uint8_t DEVADDR[4];
    extern uint8_t NWKSKEY[16];
    extern uint8_t APPSKEY[16];
    lora.set_abp(DEVADDR, NWKSKEY, APPSKEY);
    eeprom_update_byte(&ee_have_session, 23);
    eeprom_update_block(&lora.session, &ee_session, sizeof(lora.session));
  } else {
    eeprom_read_block(&lora.session, &ee_session, sizeof(lora.session));
  }

  /*
  lora.send(&payload, NULL, 7, 0); // manually set datarate
  eeprom_update_word(&ee_session.counter, lora.session.counter);
  */

  if (!have_session) print_session();

  while (1) {
    button.is_pressed(&callback_button);

    clock.sleep_for(205); // 8/32768*1000*205 = 50.048828125ms

  }
#endif

}

