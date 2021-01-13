#include <avr/io.h>
#include <string.h>
#include "lorawan_struct.h"
#include "cmac.h"
#include "uart.h"
#include "aes.h"

/*
 * prepend B_0 block to message to calculate cmac/mic with data
 * according to [lorawan](https://hackmd.io/@hVCY-lCeTGeM0rEcouirxQ/S1kg6Ymo-?type=view#MIC---Message-Integrity-Code1)
 *
 * to calculate mic for a message
 * cmac = aes128_cmac(NwkSKey, B_0 | msg)
 *
 * b0 is 16bytes longer
 */
void aes128_b0(const Packet *packet, const uint16_t counter, const uint8_t direction, const uint8_t *dev_addr, Packet *b0) {
  b0->data[0] = 0x49;
  b0->data[5] = direction;
  for (uint8_t i=0; i<4; i++) b0->data[6+i] = dev_addr[3-i];
  b0->data[10] = (counter & 0x00ff);
  b0->data[11] = ((counter >> 8) & 0x00ff);
  b0->data[15] = packet->len; // PHYPayload

  for (uint8_t i=0; i<packet->len; i++) {
    b0->data[16+i] = packet->data[i];
  }
}

/*
 * calculate cmac
 * https://tools.ietf.org/html/rfc4493
 *
 * mic are 4 msb bytes from cmac
 */
void aes128_mic(const uint8_t *key, const Packet *packet, Packet *mic) {
  uint8_t k1[16] = {0};
  uint8_t k2[16] = {0};

  aes128_generate_subkeys(key, k1, k2);

  uint8_t number_of_blocks = packet->len / 16; // number of blocks-1 (used for iteration)
  uint8_t last_block_size = packet->len % 16;

  uint8_t data[16] = {0};
  uint8_t current_block[16] = {0};
  for (uint8_t block_count=0; block_count<number_of_blocks; block_count++) {
    // for (uint8_t i=0; i<16; i++) current_block[i] = packet->data[16*block_count+i]; // get next block
    aes128_copy_array(current_block, packet->data, 16, 16*block_count); // get next block
    // y = x ^ M_i; (y is like a tmp variable)
    // x = aes(K, y);
    aes128_xor(current_block, data);
    aes128_encrypt(key, current_block);
    aes128_copy_array(data, current_block); // copy processed block to data
  }

  // handle last block
  aes128_copy_array(current_block, packet->data, last_block_size, 16*number_of_blocks); // copy remaining values from packet

  if (last_block_size == 0) { // last block is complete
    aes128_xor(current_block, k1);
  } else {
    // padding: fill up last block to 16 before processing
    current_block[last_block_size] = 0x80;
    for (uint8_t i=last_block_size+1; i<16; i++) {
      current_block[i] = 0x00;
    }
    aes128_xor(current_block, k2);
  }

  aes128_xor(current_block, data);
  aes128_encrypt(key, current_block);
  // aes128_copy_array(data, current_block); // copy processed block to data

  for (uint8_t i=0; i<mic->len; i++) {
    mic->data[i] = current_block[i];
  }
}

void aes128_generate_subkeys(const uint8_t *key, uint8_t *key1, uint8_t *key2) {
  aes128_encrypt(key, key1);
  uint8_t msb = (key1[0] & 0x80) ? 1:0;

  aes128_shift_left(key1);
  if (msb) key1[15] ^= 0x87;

  // copy key1 to key2
  for (uint8_t i=0; i<16; i++) key2[i] = key1[i];

  msb = (key2[0] & 0x80) ? 1:0;
  aes128_shift_left(key2);
  if (msb) key2[15] ^= 0x87;
}
