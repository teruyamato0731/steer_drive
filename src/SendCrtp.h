#ifndef RCT_SEND_CRTP_H_
#define RCT_SEND_CRTP_H_

#include <mbed.h>

template<class P, CAN& can>
struct SendCrtp {
  bool send() {
    return send(reinterpret_cast<P*>(this)->msg());
  }
  template<std::size_t N>
  bool send(const std::array<CANMessage, N>& msg) {
    return send(msg, std::make_index_sequence<N>());
  }
  template<std::size_t N, size_t... I>
  bool send(const std::array<CANMessage, N>& msg, std::index_sequence<I...>) {
    return (... && send(msg[I]));
  }
  bool send(const CANMessage& msg) {
    return can.write(msg);
  }
};

#endif  /// RCT_SEND_CRTP_H_
