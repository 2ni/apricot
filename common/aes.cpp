#include <avr/pgmspace.h>
#include <string.h>

#include "aes.h"

/*
 * input: key
 * input/output: data
 */
void aes128_encrypt(const uint8_t *key, uint8_t *data) {
  uint8_t number_of_rounds = 10;
  uint8_t round_key[16];
  uint8_t state[4][4];

  //  copy input to state array
  for (uint8_t column=0; column<4; column++) {
    for (uint8_t row=0; row<4; row++) {
      state[row][column] = data[row + (column << 2)];
    }
  }

  // copy key to round key
  memcpy(&round_key[0], &key[0], 16);

  aes128_add_round_key(round_key, state);

  // perform 9 full rounds with mixed columns
  for (uint8_t round=1; round<number_of_rounds; round++ ) {
    // perform byte substitution with S table
    for (uint8_t column=0 ; column<4 ; column++) {
      for (uint8_t row=0 ; row<4; row++) {
        state[row][column] = aes128_sub_byte( state[row][column] );
      }
    }

    aes128_shift_rows(state);
    aes128_mix_columns(state);
    aes128_calculate_round_key(round, round_key);
    aes128_add_round_key(round_key, state);
  }

  // perform byte substitution with S table whitout mix columns
  for (uint8_t column=0; column<4; column++) {
    for (uint8_t row=0; row<4; row++ ) {
      state[row][column] = aes128_sub_byte(state[row][column]);
    }
  }

  aes128_shift_rows(state);
  aes128_calculate_round_key(number_of_rounds, round_key);
  aes128_add_round_key(round_key, state);

  // copy the state into the data array
  for (uint8_t column=0; column<4; column++ ) {
    for (uint8_t row=0; row<4; row++ ) {
      data[row + (column << 2)] = state[row][column];
    }
  }
}

void aes128_add_round_key(uint8_t *round_key, uint8_t (*state)[4]) {
  for (uint8_t col=0; col<4; col++) {
    for (uint8_t row=0; row<4; row++) {
      state[row][col] ^= round_key[row + (col << 2)];
    }
  }
}

uint8_t aes128_sub_byte(uint8_t byte) {
  //return AES_BOX [ ((byte >> 4) & 0x0F) ] [ ((byte >> 0) & 0x0F) ]; // original
  return pgm_read_byte(&(AES_BOX [((byte >> 4) & 0x0F)] [((byte >> 0) & 0x0F)]));
}

void aes128_shift_rows(uint8_t (*state)[4]) {
  uint8_t buffer;

  // store 1st byte in buffer
  buffer      = state[1][0];
  // shift all bytes
  state[1][0] = state[1][1];
  state[1][1] = state[1][2];
  state[1][2] = state[1][3];
  state[1][3] = buffer;

  buffer      = state[2][0];
  state[2][0] = state[2][2];
  state[2][2] = buffer;
  buffer      = state[2][1];
  state[2][1] = state[2][3];
  state[2][3] = buffer;

  buffer      = state[3][3];
  state[3][3] = state[3][2];
  state[3][2] = state[3][1];
  state[3][1] = state[3][0];
  state[3][0] = buffer;
}

void aes128_mix_columns(uint8_t (*state)[4]) {
  uint8_t col;
  uint8_t a[4], b[4];


  for(col=0; col<4; col++) {
    for (uint8_t row=0; row<4; row++) {
      a[row] =  state[row][col];
      b[row] = (state[row][col] << 1);

      if ((state[row][col] & 0x80) == 0x80) {
        b[row] ^= 0x1B;
      }
    }

    state[0][col] = b[0] ^ a[1] ^ b[1] ^ a[2] ^ a[3];
    state[1][col] = a[0] ^ b[1] ^ a[2] ^ b[2] ^ a[3];
    state[2][col] = a[0] ^ a[1] ^ b[2] ^ a[3] ^ b[3];
    state[3][col] = a[0] ^ b[0] ^ a[1] ^ a[2] ^ b[3];
  }
}

void aes128_calculate_round_key(uint8_t round, uint8_t *round_key) {
  uint8_t b, rcon;
  uint8_t temp[4];

  // calculate rcon
  rcon = 0x01;
  while(round != 1) {
    b = rcon & 0x80;
    rcon = rcon << 1;

    if (b == 0x80) {
      rcon ^= 0x1b;
    }
    round--;
  }

  // calculate first temp
  // copy laste byte from previous key and subsitute the byte, but shift the array contents around by 1.
  temp[0] = aes128_sub_byte(round_key[12 + 1]);
  temp[1] = aes128_sub_byte(round_key[12 + 2]);
  temp[2] = aes128_sub_byte(round_key[12 + 3]);
  temp[3] = aes128_sub_byte(round_key[12 + 0]);

  // xor with rcon
  temp[0] ^= rcon;

  // calculate new key
  for (uint8_t i=0; i<4; i++) {
    for (uint8_t j=0; j<4; j++) {
      round_key[j + (i << 2)] ^= temp[j];
      temp[j] = round_key[j + (i << 2)];
    }
  }
}

void aes128_copy_array(uint8_t *dest, uint8_t *src, uint8_t len, uint8_t offset) {
  for (uint8_t i=0; i<len; i++) dest[i] = src[offset+i];
}

void aes128_xor(uint8_t *new_data, uint8_t *old_data) {
  for (uint8_t i = 0; i < 16; i++) {
    new_data[i] = new_data[i] ^ old_data[i];
  }
}

void aes128_shift_left(uint8_t *data) {
  uint8_t overflow = 0;

  for (uint8_t i=0; i<16; i++) {
    // check for overflow on next byte except for the last byte
    if (i < 15) {
      overflow = (data[i+1] & 0x80) ? 1:0;
    } else {
      overflow = 0;
    }

    // shift one left
    data[i] = (data[i] << 1) + overflow;
  }
}
