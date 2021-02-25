/*
 * unit test cases for lorawan
 */
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "test.h"
#include "struct.h"
#include "pins.h"
#include "cmac.h"
#include "aes.h"
#include "lorawan.h"
#include "uart.h"
#include "tests_lora.h"

Test_Result tests_lora() {
  printf("%s\n", __func__);

  uint16_t number_of_tests = 0;
  uint16_t number_of_passed = 0;
  Packet packet;
  Packet expected;

  uint16_t counter = 0x1234;
  uint8_t devaddr[4] = { 0x11, 0x12, 0x13, 0x14 };
  uint8_t nwkskey[16] = { 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };
  uint8_t appskey[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };

  LORAWAN lora_obj(0, 0); // no adr, no linecheck
  lora_obj.reset_session(); // set eg frequencies and chmask
  lora_obj.set_abp(devaddr, nwkskey, appskey, counter);

  // chmask frequency position
  lora_obj.session.chmask = 0b00101101; // pos 0, 2, 3, 5
  lora_obj.session.frequencies[3] = 8671000;
  lora_obj.session.frequencies[5] = 8675000;
  const uint8_t chmask_len = 10;
  uint8_t chmask_positions[chmask_len];
  uint8_t chmask_positions_expected[chmask_len] = { 2, 3, 5, 0, 2, 3, 5, 0, 2, 3 };
  uint32_t frqs[chmask_len];
  expected.len = chmask_len;
  expected.data = chmask_positions_expected;
  for (uint8_t x=0; x<chmask_len; x++) {
    chmask_positions[x] = lora_obj.get_next_frq_pos();
    frqs[x] = lora_obj.session.frequencies[chmask_positions[x]];
  }
  packet.len = chmask_len;
  packet.data = chmask_positions;
  number_of_passed += validate("next frequency position", &expected, &packet);
  number_of_tests++;

  uint32_t frqs_expected[chmask_len] = {8685000, 8671000, 8675000, 8681000, 8685000, 8671000, 8675000, 8681000, 8685000, 8671000};
  Test_Outcome test = PASS;
  for (uint8_t x=0; x<chmask_len; x++) {
    if (frqs[x] != frqs_expected[x]) {
      test = FAIL;
      break;
    }
  }
  print_test("next frequency", test);
  if (test == FAIL) {
    printf("   expected: ");
    printarray(frqs_expected, chmask_len);
    printf("   got     : ");
    printarray(frqs, chmask_len);
  }
  number_of_passed += test;
  number_of_tests++;

  // cipher (alters packet!)
  const uint8_t lc = 3;
  uint8_t dc[lc] = { 0x61, 0x62, 0x63 };
  packet.data=dc;
  packet.len=lc;
  uint8_t direction = 0;
  lora_obj.cipher(&packet, lora_obj.session.counter, direction, 1, lora_obj.session.devaddr);
  uint8_t edatac[lc] = { 0x2e, 0xe5, 0x20 };
  expected.data = edatac;
  expected.len = lc;

  number_of_passed += validate("lora cipher", &expected, &packet);
  number_of_tests++;

  // B0 appending
  const uint8_t len = 4;
  uint8_t data[len] = { 0x01, 0x02, 0x03, 0x04 };
  packet.data=data;
  packet.len=len;
  direction = 1;

  const uint8_t blocklen = len + 16;
  uint8_t blockdata[blocklen] = {0};
  Packet block = { .data=blockdata, .len=blocklen };

  aes128_b0(&packet, counter, direction, devaddr, &block);
  const uint8_t elen = 20;
  uint8_t edata[elen] = { 0x49, 0x00, 0x00, 0x00, 0x00, 0x01, 0x14, 0x13, 0x12, 0x11, 0x34, 0x12, 0x00, 0x00, 0x00, 0x04, 0x01, 0x02, 0x03, 0x04 };
  expected.data = edata;
  expected.len = elen;
  number_of_tests++;
  number_of_passed += validate("prepend B0", &expected, &block);

  // mic
  uint8_t dmic[4] = {0};
  Packet mic = { .data=dmic, .len=4 };
  aes128_mic(lora_obj.session.nwkskey, &block, &mic);
  const uint8_t mlen = 4;
  uint8_t mdata[mlen] = { 0x6a, 0x01, 0xbf, 0x55 };
  expected.data = mdata;
  expected.len = mlen;
  number_of_tests++;
  number_of_passed += validate("mic", &expected, &mic);

  // lora packet
  const uint8_t pl = 3;
  uint8_t pd[pl] = { 0x61, 0x62, 0x63 };
  Packet payload = { .data=pd, .len=pl };
  const uint8_t ll = pl+13;
  uint8_t ld[ll] = {0};
  Packet lora = { .data=ld, .len=ll };
  lora_obj.create_package(&payload, &lora);
  uint8_t edl[ll] = { 0x40, 0x14, 0x13, 0x12, 0x11, 0x00, 0x34, 0x12, 0x01, 0x2e, 0xe5, 0x20, 0xf4, 0x80, 0xfd, 0x1e };
  expected.data = edl;
  expected.len = ll;
  number_of_tests++;
  number_of_passed += validate("lora package", &expected, &lora);

  // check input data packet
  const uint8_t l_phy = 14;
  uint8_t d_phy[l_phy] = { 0x60, 0x11, 0x34, 0x01, 0x26, 0x10, 0x35, 0x00, 0x01, 0x74, 0x2d, 0x08, 0x6a, 0xc0 };
  Packet phy = { .data=d_phy, .len=l_phy };
  phy.len -= 4;
  uint16_t phy_counter = phy.data[7];
  phy_counter = (phy_counter << 8) + phy.data[6];
  direction = 1;
  uint8_t lenb0 = phy.len+16;
  uint8_t datab0[lenb0];
  memset(datab0, 0, lenb0);
  Packet b0 = { .data=datab0, .len=lenb0 };
  aes128_b0(&phy, phy_counter, direction, devaddr, &b0);

  aes128_mic(lora_obj.session.nwkskey, &b0, &mic);
  // printarray(b0.data, b0.len);
  number_of_tests++;
  expected.data = &phy.data[l_phy-4];
  expected.len = 4;
  number_of_passed += validate("lora rx data package", &expected, &mic);

  // summary
  Test_Result result = { .total=number_of_tests, .passed=number_of_passed };
  return result;
}
