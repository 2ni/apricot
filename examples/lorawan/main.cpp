#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#include "uart.h"
#include "pins.h"
#include "mcu.h"
#include "lorawan.h"
#include "keys.h"
#include "sleep.h"

#define OTAA

LORAWAN lora;
uint8_t  EEMEM ee_have_session;
Lora_session EEMEM ee_session;

int main(void) {
  mcu_init();
  lora.init();

  uint8_t have_session = eeprom_read_byte(&ee_have_session) == 0xff ? 0 : 1;
  if (have_session) {
    DL("load session from eeprom");
    eeprom_read_block(&lora.session, &ee_session, sizeof(lora.session));
  }

  uint8_t len = 3;
  uint8_t data[len] = { 0x61, 0x62, 0x63 };
  Packet payload = { .data=data, .len=len };

  uint8_t rx_len = 64;
  uint8_t rx_data[rx_len] = {0};
  Packet rx_packet = { .data=rx_data, .len=rx_len };

#ifdef OTAA
  if (!have_session) {
    extern uint8_t DEVEUI[8];
    extern uint8_t APPEUI[8];
    extern uint8_t APPKEY[16];
    lora.set_otaa(DEVEUI, APPEUI, APPKEY);

    if (lora.join() == OK) {
      have_session = 1;
      eeprom_update_byte(&ee_have_session, have_session);
      eeprom_update_block(&lora.session, &ee_session, sizeof(lora.session));
    } else {
      DL("join failed");
    }
  }

  if (have_session) {
    uart_arr("appskey", lora.session.appskey, 16);
    uart_arr("nwkskey", lora.session.nwkskey, 16);
    uart_arr("devaddr", lora.session.devaddr, 4);
    // DF("datarate: %u\n", lora.session.datarate);
    DF("counter: %u\n", lora.session.counter);
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
#else
  if (!have_session) {
    extern uint8_t DEVADDR[4];
    extern uint8_t NWKSKEY[16];
    extern uint8_t APPSKEY[16];
    lora.set_abp(DEVADDR, NWKSKEY, APPSKEY);
    have_session = 1;
    eeprom_update_byte(&ee_have_session, have_session);
  }
  Status status = lora.send(&payload, &rx_packet, 7, 0); // datarate is not set in abp mode
  eeprom_update_word(&ee_session.counter, lora.session.counter);
  if (rx_packet.len) {
    uart_arr("received message", rx_packet.data, rx_packet.len);
  } else {
    DL("no data received");
  }
#endif

  DL("done.");

  while (1);
}
