#include "queue.h"

QUEUE::QUEUE() {
  head = 0;
  tail = 0;
}

uint8_t QUEUE::empty() {
  return head == tail;
}

uint8_t QUEUE::full() {
  return head == ((tail+1) % QUEUE_SIZE);
}

void QUEUE::push(DCC::PACKET &packet) {
  if (full()) return;

  packets[tail] = packet;
  tail = (tail+1) % QUEUE_SIZE;
}

void QUEUE::pull(DCC::PACKET &packet) {
  if (empty()) {
    packet.len = 0;
    return;
  }

  packet = packets[head];
  head = (head+1) % QUEUE_SIZE;
}
