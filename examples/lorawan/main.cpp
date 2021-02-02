#include <util/delay.h>
#include <avr/io.h>
#include <avr/eeprom.h>

#include "uart.h"
#include "pins.h"
#include "mcu.h"
#include "lorawan.h"
#include "keys.h"
#include "sleep.h"
#include "touch.h"

#define OTAA

#define SLEEP_TIME 50
// to avoid complex multiplications we calculate the needed timer count beforehand and use the internal function _s_sleep()
#define SLEEP_PER UINT32_C(SLEEP_TIME)*32768/1000

LORAWAN lora;
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
}

void join() {
  extern uint8_t DEVEUI[8];
  extern uint8_t APPEUI[8];
  extern uint8_t APPKEY[16];
  lora.set_otaa(DEVEUI, APPEUI, APPKEY);

  if (lora.join() == OK) {
    have_session = 1;
    eeprom_update_byte(&ee_have_session, have_session);
    eeprom_update_block(&lora.session, &ee_session, sizeof(lora.session));
    print_session();
  } else {
    DL("join failed");
    while (1);
  }
}

int main(void) {
  mcu_init();
  lora.init();

  TOUCH button(&PB7);

  uint8_t have_session = eeprom_read_byte(&ee_have_session) == 0xff ? 0 : 1;
  if (have_session) {
    DL("load session from eeprom");
    eeprom_read_block(&lora.session, &ee_session, sizeof(lora.session));
    print_session();
  }

  uint8_t len = 3;
  uint8_t data[len] = { 0x61, 0x62, 0x63 };
  Packet payload = { .data=data, .len=len };

  uint8_t rx_len = 64;
  uint8_t rx_data[rx_len] = {0};
  Packet rx_packet = { .data=rx_data, .len=rx_len };

  uint8_t is_pressed;

#ifdef OTAA
  if (!have_session) join();

  while (1) {
    is_pressed = button.is_pressed(&pins_led);
    if (is_pressed) {
      // DF("pressed: 0x%02x\n", is_pressed & ~touch_is_pressed_bm);
      // set led when long touch was reached
      if (is_pressed & touch_long_bm) {
        pins_set(&pins_led, 1);
      } else if (is_pressed & touch_verylong_bm) {
        pins_set(&pins_led, 0);
      }
    }

    // released, if MSB not set but lower bits set, eg 0x02
    if (is_pressed && (~is_pressed & touch_is_pressed_bm)) {
      pins_set(&pins_led, 0);
      if (is_pressed == touch_short_bm) {
        DF("counter      : %u\n", lora.session.counter);
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
      } else if (is_pressed == touch_long_bm) {
        have_session = 0;
        join();
      } else if (is_pressed == touch_verylong_bm) {
        print_session();
      }
    }

    // do not sleep if pressed, as we need to measure time (whic is on hold while sleeping)
    if (!is_pressed) {
      _sleep(SLEEP_PER, 0); // 1638 (precalculated) vs 1650 using sleep_ms(50);
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
