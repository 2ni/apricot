/*
 * unit test cases for lorawan
 */
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "test.h"
#include "pins.h"
#include "lorawan.h"
#include "tests_mac.h"
#include "uart.h"

Test_Result tests_mac() {
  printf("%s\n", __func__);

  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;
  Packet packet;
  Packet expected;

  LORAWAN lora_obj(0, 0); // no adr, no linecheck

  // LORA_MAC_LINKADR: SF9, 10dB, different channels, no general channel settings
  //   datarate[7:4]: 0=SF12, 1=SF11, ..., 5=SF7
  //   txpower[3:0]: 0=max, 1=max-2db, 2=max-4db, ...
  //   ChMask(2): 0: channel 1, 1: channel 2, ... 15: channel 16
  //   Redundancy(1)
  // LORA_MAC_DUTYCYCLE: 50% (not implemented, tests offset)
  // LORA_MAC_RXPARAM: RX2DataRate SF8, RX1DROffset 2, no frq (not implemented)
  //   RX1DROffset[6:4]
  //   RX2DataRate[3:0]
  const uint8_t rx_cmds_l = 12;
  uint8_t rx_cmds_d[rx_cmds_l] = { LORA_MAC_LINKADR, 0x33, 0x00, 0xff, 0x01, LORA_MAC_DUTYCYCLE, 0x01, LORA_MAC_RXPARAM, 0x24, 0x00, 0x00, 0x00};
  Packet rx_cmds = { .data=rx_cmds_d, .len=rx_cmds_l };
  lora_obj.process_cmds(&rx_cmds);

  number_of_passed += validate("MAC LINKADR datarate", 0x09, lora_obj.session.txdatarate);
  number_of_passed += validate("MAC LINKADR power", 10, lora_obj.session.txpower);
  number_of_passed += validate("MAC LINKADR chmask", 0x00ff, lora_obj.session.chmask);

  // validate tx queue mac commands
  const uint8_t l = 2;
  uint8_t expected_queue_d[l] = { LORA_MAC_LINKADR, 0x07 }; // ack datarate, txpower, chmask
  Packet expected_queue = { .data=expected_queue_d, .len=l };
  uint8_t queue_d[l] = {0};
  for (uint8_t i=0; i<l; i++) {
    queue_d[i] = lora_obj.queue_cmd_tx[i];
  }
  Packet queue = { .data=queue_d, .len=l };
  number_of_passed += validate("MAC LINKADR txqueue", &expected_queue, &queue);

  number_of_passed += validate("MAC RXPARAM rx2datarate", 8, lora_obj.session.rx2datarate);
  number_of_passed += validate("MAC RXPARAM rx1droffset", 2, lora_obj.session.rxoffset);

  number_of_tests+=6;

  // summary
  Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}
