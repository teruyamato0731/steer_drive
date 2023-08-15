#ifndef RCT_AMT21_H_
#define RCT_AMT21_H_

#include <cstdint>

struct Amt21 {
  static constexpr int rotate = 4096;

  uint8_t address;
  uint16_t pos;
  uint16_t turns;

  int get_pos() const {
    return rotate * get_turns() + pos;
  }
  int16_t get_turns() const {
    int16_t sign_bit = turns & 0x2000;
    sign_bit = sign_bit << 2 | sign_bit << 1;
    return sign_bit | turns;
  }
  bool read_pos(const uint16_t msg) {
    bool res = is_valid(msg);
    if(res) pos = (msg & 0x3fff) >> 2;
    return res;
  }
  bool read_turns(const uint16_t msg) {
    bool res = is_valid(msg);
    if(res) turns = msg & 0x3fff;
    return res;
  }
  static bool is_valid(uint16_t raw_data) {
    bool k1 = raw_data >> 15;
    bool k0 = raw_data >> 14 & 1;
    raw_data <<= 2;
    do {
      k1 ^= raw_data & 0x8000;          // even
      k0 ^= (raw_data <<= 1) & 0x8000;  // odd
    } while(raw_data <<= 1);
    return k0 && k1;
  }
};

#endif  // RCT_AMT21_H_
