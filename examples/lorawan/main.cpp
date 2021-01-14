#include <util/delay.h>
#include <avr/io.h>

#include "uart.h"
#include "pins.h"
#include "mcu.h"
#include "lorawan.h"
#include "keys.h"

#define OTAA

int main(void) {
  mcu_init();
  LORAWAN lora;

  lora.init();

  while(1);

  uint8_t len = 3;
  uint8_t data[len] = { 0x61, 0x62, 0x63 };
  Packet payload = { .data=data, .len=len };

  uint8_t rx_len = 64;
  uint8_t rx_data[rx_len] = {0};
  Packet rx_packet = { .data=rx_data, .len=rx_len };

#ifdef OTAA
  extern uint8_t DEVEUI[8];
  extern uint8_t APPEUI[8];
  extern uint8_t APPKEY[16];
  lora.set_otaa(DEVEUI, APPEUI, APPKEY);

  if (lora.join(1)) { // scan all
    uart_arr("appskey", lora.session.appskey, 16);
    uart_arr("nwkskey", lora.session.nwkskey, 16);
    uart_arr("devaddr", lora.session.devaddr, 4);
    DF("datarate: %u\n", lora.session.datarate);
    if (lora.send(&payload, &rx_packet) == OK) {
      uart_arr("received message", rx_packet.data, rx_packet.len);
    }
  } else {
    DL(NOK("join failed"));
  }
#else
  extern uint8_t DEVADDR[4];
  extern uint8_t NWKSKEY[16];
  extern uint8_t APPSKEY[16];
  lora.set_abp(DEVADDR, NWKSKEY, APPSKEY);
  if (lora.send(&payload, 7, &rx_packet) == OK) { // datarate is not set in abp mode
    uart_arr("received message", rx_packet.data, rx_packet.len);
  }
#endif

  DL("done.");

  while (1);
}
