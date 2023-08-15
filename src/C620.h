#ifndef RCT_C620_H_
#define RCT_C620_H_

#include <mbed.h>

#include <array>

#include "swap_endian.h"

struct C620 {
  static constexpr int max = 16384;

  struct RxPacket {
    uint16_t angle;
    int16_t rpm;
    int16_t ampere;
    uint8_t temp;
    uint8_t padding;

    void set(const uint8_t data[8]) {
      angle = uint16_t(data[0] << 8 | data[1]);
      rpm = int16_t(data[2] << 8 | data[3]);
      ampere = int16_t(data[4] << 8 | data[5]);
      temp = data[6];
    }
  };

  int16_t pwm[8];
  RxPacket data[8];

  std::array<CANMessage, 2> msg() const {
    auto buf = swap_endian(pwm);
    const uint8_t* data = reinterpret_cast<const uint8_t*>(buf.data());
    return {CANMessage{0x200, data, 8}, CANMessage{0x199, data + 8, 8}};
  }
  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8 && 0x200 <= msg.id && msg.id <= 0x208) {
      data[msg.id - 0x201u].set(msg.data);
    }
  }
};

#endif  /// RCT_C620_H_
