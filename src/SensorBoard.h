#ifndef RCT_SENSOR_BOARD_H_
#define RCT_SENSOR_BOARD_H_

#include <mbed.h>

struct SensorBoard {
  void set_forward(const CANMessage& msg) {
    time_us = uint64_t{msg.data[4]} << 32 | msg.data[3] << 24 | msg.data[2] << 16 | msg.data[1] << 8 | msg.data[0];
    lim = msg.data[5];
    enc[4] = msg.data[7] << 8 | msg.data[6];
  }
  void set_backward(const CANMessage& msg) {
    enc[3] = msg.data[1] << 8 | msg.data[0];
    enc[2] = msg.data[3] << 8 | msg.data[2];
    enc[1] = msg.data[5] << 8 | msg.data[4];
    enc[0] = msg.data[7] << 8 | msg.data[6];
  }
  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8) {
      if(msg.id == id[0]) {
        set_forward(msg);
      } else if(msg.id == id[1]) {
        set_backward(msg);
      }
    }
  }
  unsigned id[2];

  uint64_t time_us = {};
  uint8_t lim = {};
  volatile int16_t enc[5] = {};
};

#endif  // RCT_SENSOR_BOARD_H_
