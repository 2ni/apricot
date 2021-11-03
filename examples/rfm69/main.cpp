#include <util/delay.h>
#include <string.h>

#include "pins.h"
#include "uart.h"
#include "mcu.h"
#include "rfm69.h"

#define NODE 2
#define GATEWAY 99
#define NETWORK 33

int main(void) {
  mcu_init();

  RFM69 rfm69;
  uint8_t counter = 0;
  uint8_t len;
  char msg[9];

  uint8_t version = rfm69.init(NODE, NETWORK);
  if (!version) {
    DL("rfm69 init failed");
  } else {
    DF("RFM69 Version: 0x%02x\n", version);
  }

  RFM69::Packet response;

  /*
  DL("Waiting for data...");
  while (1) {
    if (rfm69.listen(&response, 0)) { // no timeout
      DF("received: '%s' (%idBm)\n", response.message, response.rssi);
    }
  }
  //*/

  /*
  while (1) {
    DF("sending %03u | ", counter);
    len = sprintf(msg, "%u", counter);
    rfm69.send_frame(GATEWAY, msg, len, 1, 0);
    if (rfm69.listen(&response)) {
      DF("ack (%idBm)", response.rssi);
    }
    DL("");
    counter++;
    clock.sleep_for(4096);
  }
  //*/

  while (1) {
    DF("sending %03u | ", counter);
    len = sprintf(msg, "%u", counter);
    // rfm69.send(GATEWAY, msg, len);
    // DL("");
    // if (rfm69.send(GATEWAY, msg, len, response)) {
    //   DF("response: '%s'\n", response);
    // }
    if (rfm69.send_retry(GATEWAY, msg, len, &response, 2)) {
      // 0x%06lX
      DF(OK("from 0x%u: '%s' (%idBm)") "\n", response.from, response.message, response.rssi);
    } else {
      DL(NOK("no response"));
    }

    counter++;
    clock.sleep_for(8192);
  }
}

#if 0
int main(void) {
  mcu_init();

  rfm69_cs = PC3;
  rfm69_interrupt = PC4;
  //TODO check if rfm69_cs is corerctly set in rfm69 library itself

  // test spi by shorting MISO - MOSI
  /*
  spi_init();
  uint8_t test_byte = spi_transfer_byte(0x38);
  DF("test byte: 0x%02x\n", test_byte);
  */

  // test rfm69
  // () sth about pulldown on dio0 pin
  /*
  spi_init();
  uint8_t reg = read_reg(REG_VERSION); // addr: 0x10 should be 0x24
  DF("Version: 0x%02x\n", reg);
  while (1) {}
  */

  uint8_t counter = 0;
  char msg[9];
  // uint8_t ack_request = 1;
  char incoming[RF69_MAX_DATA_LEN+1];
  int16_t rssi;
  uint8_t sleep_time = 2;
  char cmd;
  uint8_t len;
  uint8_t timeout;

  uint8_t version = rfm69_init(868, NODE, NETWORK);
  if (!version) {
    DL("rfm69 init failed");
  } else {
    DF("RFM69 Version: 0x%02x\n", version);
  }

  DF("send: %u->%u, network: %u\n", NODE, TONODE, NETWORK);
  while (1) {
    DF("sending %03u | ", counter);
    len = sprintf(msg, "%u", counter);
    /*
    rfm69_send(TONODE, msg, len, 0);
    DL("");
    /*/
    timeout = rfm69_send_retry(TONODE, msg, len, 2);
    if (timeout) {
      D(" | waiting | ");
      rssi = rfm69_get_data(incoming, RF69_MAX_DATA_LEN);
      DF(OK("received in %ums") " \"%s\" (%idBm)\n", timeout, incoming, rssi);
    } else {
     DL(NOK("no data"));
    }
    // */
    counter++;
    clock.sleep_for(4096);
  }

  while (1) {
    // node
    len = sprintf(msg, "Hello: %u", (counter++)%10);

    DF("send_retry: %u->%u (%u): '%s'", NODE, TONODE, NETWORK, msg);
    if (rfm69_send_retry(TONODE, msg, len, 2)) {
      rssi = rfm69_get_data(incoming, RF69_MAX_DATA_LEN);
      sscanf(incoming, " %c:%u", &cmd, (unsigned int *)&sleep_time);
      if (cmd=='s') {
        // DF("((%c, %u))", cmd, value);
      }
      /*
      char * token;
      token = strtok(incoming, "|");
      while (token) {
      }
      */
      DF(" ...ok with response: '%s' [RSSI: %i] [sleep: %u]\n", incoming, rssi, sleep_time);
    } else {
      DL(" ...failed");
    }

    clock.sleep_for(sleep_time * 4096); // 4096 = 1sec with prescale of 8
  }
}
#endif
