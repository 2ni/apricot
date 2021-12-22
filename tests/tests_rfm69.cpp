#include <stdio.h>
#include <string.h>

#include "test.h"
#include "tests_rfm69.h"
#include "rfm69wrapper.h"
#include "rfm69.h"
#include "uart.h"

namespace TYPES {
  typedef enum {
    DBG = 0x00,
    VCC = 0x01,
    TEMP = 0x02,
    RSSI = 0x03,
    HUM = 0x08
  } Types;
}

Test_Result tests_rfm69() {
  printf("%s\n", __func__);

  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;

  RFM69WRAPPER rf;

  // tests
  uint8_t type, len;

  rf.decode_first_byte(0x32, &type, &len);
  number_of_passed += validate("decode first byte (type)", 0x03, type);
  number_of_passed += validate("decode first byte (len)", 0x02, len);
  number_of_passed += validate("encode first byte", 0x32, rf.encode_first_byte(type, len));
  number_of_tests += 3;

  RFM69::Packet response;
  response.len = 8;
  // dbg (0x01, 123), rssi (limit|request, 95), vcc (0x12, 317)
  static const uint8_t temp[8] = { 0x01, 123, 0x32, 1<<7|1<<5, 95, 0x12, 0x01, 0x3d };
  memcpy(response.payload, temp, sizeof temp);
  RFM69WRAPPER::WPacket wpacket;

  uint8_t i = 0;
  uint8_t number_of_packets = 0;
  while (rf.decode(&i, &response, &wpacket)) {
    switch (wpacket.type) {
      case TYPES::DBG:
        number_of_passed += validate("type dbg", 123, wpacket.payload[0]);
        break;
      case TYPES::VCC:
        number_of_passed += validate("type vcc", 317, wpacket.payload[0]<<8 | wpacket.payload[1]);
    }
    number_of_packets++;
  }

  number_of_passed += validate("number of packets", 3, number_of_packets);

  number_of_tests += 3;

  // final
  Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}
