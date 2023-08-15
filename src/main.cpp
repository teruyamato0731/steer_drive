#include <Kernel.h>
#include <Pid.h>
#include <SteerDrive.h>
#include <mbed.h>

#include <algorithm>
#include <complex>
#include <utility>

#include "Amt21.h"
#include "C620.h"
#include "PollWait.h"
#include "Rs485.h"
#include "SendCrtp.h"
#include "SensorBoard.h"

// const variable
constexpr auto dc_id = 5;
constexpr auto enc_rot = Amt21::rotate * 3;

// prototype
void wait_can();

// IO
BufferedSerial pc{USBTX, USBRX, 115200};
Rs485 rs485{PB_6, PA_10, (int)2e6, PC_0};
CAN can1{PA_11, PA_12, (int)1e6};
CAN can2{PB_12, PB_13, (int)1e6};
CANMessage msg;
Timer timer;

// struct definition
struct DCSender : SendCrtp<DCSender, can1> {
  static constexpr int max = INT16_MAX * 0.7;
  int16_t pwm[4];
  auto msg() {
    return CANMessage{dc_id, (uint8_t*)pwm, 8};
  }
};
struct : Amt21 {
  // request
  bool send_read_pos() {
    rs485.uart_transmit({address});
    if(uint16_t receive; rs485.uart_receive(&receive, sizeof(receive), 10ms)) {
      read_pos(receive);
      return true;
    }
    return false;
  }
  bool send_read_turns() {
    rs485.uart_transmit({address + 1});
    if(uint16_t receive; rs485.uart_receive(&receive, sizeof(receive), 10ms)) {
      read_turns(receive);
      return true;
    }
    return false;
  }
  void send_reset() {
    rs485.uart_transmit({address + 2, 0x75});
  }
} amt[] = {{0x50}, {0x54}, {0x58}, {0x5C}};
constexpr rct::PidGain drive_gain{1.2, 0.3};
constexpr rct::PidGain gain{36.5, 3, 0.005};
struct SteerUnit {
  auto calc_pid(const int rpm, const int pos, const std::chrono::microseconds& delta_time) {
    float drive = pid_drive.calc(target_rpm, rpm, delta_time);
    drive = std::clamp(drive, -1.0f * C620::max, 1.0f * C620::max);
    auto steer = pid_steer.calc(target_pos, pos, delta_time);
    steer = -std::clamp(steer, -0.7f * DCSender::max, 0.7f * DCSender::max);
    return std::tuple{drive, steer};
  }
  rct::Pid<float> pid_drive = {drive_gain};
  rct::Pid<float> pid_steer = {gain};
  int target_rpm;
  int target_pos;
};

// Control
SensorBoard sensor_board{9u, 10u};
struct : C620, SendCrtp<C620, can2> {
} dji{};
DCSender dc_sender{};
struct SteerOdom {
  static constexpr int N = 4;
  void integrate(std::complex<float> (&dif_val)[N]) {
    for(int i = 0; i < N; ++i) {
      auto rotated = dif_val[i] * std::polar<float>(1, -M_PI / N * (2 * i + 3));
      pos_.x_milli += rotated.real();
      pos_.y_milli += rotated.imag();
      pos_.ang_rad += dif_val[i].real();
    }
  }
  auto& get() const& {
    return pos_;
  }
 private:
  rct::Coordinate pos_;
} odom{};
SteerUnit unit[4] = {};
rct::SteerDrive<4> steer{[](std::array<std::complex<float>, 4> cmp) {
  for(int i = 0; i < 4; ++i) {
    int new_tag_pos = enc_rot / 2 / M_PI * arg(cmp[i]);
    int offset = new_tag_pos - amt[i].get_pos();
    int r = std::round(2.0 * offset / enc_rot);
    int drive_dir = 2 * (r % 2 == 0) - 1;
    unit[i].target_rpm = abs(cmp[i]) * 9000 * drive_dir;  // max 9000rpm
    unit[i].target_pos = new_tag_pos - r * (enc_rot / 2);
  }
}};
struct Controller {
  uint8_t button[2];
  int8_t stick[4];  // LX,LY,RX,RY

  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.id == 15u) {
      memcpy(this, msg.data, sizeof(Controller));
      for(auto& e: stick) {
        e -= 128;
        if(std::abs(e) < 15) e = 0;
      }
    }
  }
  rct::Velocity get_vel() const {
    return {stick[1] / 128.0f, -stick[0] / 128.0f, stick[2] / 128.0f * 3 / 4};
  }
} controller;

int main() {
  // put your setup code here, to run once:
  wait_can();

  printf("\nsetup\n");

  printf("enc reset... ");
  for(auto& e: amt) e.send_read_pos();
  printf("done\n");

  timer.start();
  while(1) {
    auto now = timer.elapsed_time();
    static auto pre = now - 10ms;
    static auto pre_alive = now - 100ms;

    // CANMessageの受取り
    if(can1.read(msg)) {
      sensor_board.read(msg);
      controller.read(msg);
    }
    if(can2.read(msg)) {
      dji.read(msg);
      if(0x200u < msg.id && msg.id <= 0x208u) {
        // C620の生存CK
        pre_alive = now;
      }
    }

    for(auto& e: amt) {
      e.send_read_pos();
      e.send_read_turns();
    }

    // 10msごとにCAN送信
    if(static PollWait<Kernel::Clock> wait{}; auto delta = wait(10ms)) {
      rct::Velocity vel = controller.get_vel();

      steer.move(vel);
      for(auto i = 0; i < 4; ++i) {
        if(now - pre_alive < 100ms || true) {
          // pidの計算
          // TODO encの更新に合わせる？ delta timeを
          std::tie(dji.pwm[i], dc_sender.pwm[i]) = unit[i].calc_pid(dji.data[i].rpm, amt[i].get_pos(), delta);
        } else {
          // fail時, 出力を1/2倍していく
          dji.pwm[i] /= 2;
          dc_sender.pwm[i] /= 2;
          // I値を消す
          unit[i].pid_drive.refresh();
          unit[i].pid_steer.refresh();
        }
      }
      // C620が生存なら
      if(now - pre_alive < 100ms) {
        // Odom で自己位置を推定
        std::complex<float> diff[4];
        for(auto i = 0; i < 4; ++i) {
          float rho = dji.data[i].rpm * delta.count() * 1e-6;
          float theta = 2 * M_PI / enc_rot * amt[i].pos;
          diff[i] = std::polar(rho, theta);
        }
        odom.integrate(diff);
      }

      // printf("vel:");
      // printf("%3d\t", (int)(vel.x_milli * 128));
      // printf("%3d\t", (int)(vel.y_milli * 128));
      // printf("%3d\t", (int)(vel.ang_rad * 128));
      // printf("enc:");
      // for(auto i = 0; i < 4; ++i) {
      //   printf("% 5d\t", sensor_board.enc[i]);
      // }
      // printf("zero:");
      // for(auto i = 0; i < 4; ++i) {
      //   printf("% 5d\t", unit[i].zero_pos);
      // }
      printf("pos:");
      for(auto i = 0; i < 4; ++i) {
        printf("% 4d\t", amt[i].get_pos());
      }
      printf("tag:");
      for(auto& e: unit) {
        printf("% 5d\t", e.target_pos);
      }
      printf(" dc:");
      for(auto& e: dc_sender.pwm) {
        printf("% 5d\t", e);
      }
      // printf("rpm:");
      // for(int i = 0; i < 4; ++i) {
      //   printf("% 5d\t", reader.data[i].rpm);
      // }
      // printf("tag:");
      // for(auto& e: unit) {
      //   printf("% 5d\t", e.target_rpm);
      // }
      // printf(" ac:");
      // for(int i = 0; i < 4; ++i) {
      //   printf("% 5d\t", sender.pwm[i]);
      // }

      // printf("pos:");
      // printf("% 4d\t", sensor_board.enc[2] - unit[2].zero_pos);
      // printf("tag:");
      // printf("% 5d\t", unit[2].target_pos);
      // printf(" dc:");
      // printf("% 5d\t", dc_sender.pwm[2]);

      printf("est:");
      printf("%3d\t", (int)odom.get().x_milli);
      printf("%3d\t", (int)odom.get().y_milli);
      printf("%3d\t", (int)odom.get().ang_rad);

      printf("\n");

      dc_sender.send();
      dji.send();
    }
  }
}

void wait_can() {
  bool receive[3] = {};
  while(!(receive[0] && receive[1] && receive[2])) {
    if(can1.read(msg)) {
      sensor_board.read(msg);
      controller.read(msg);
      receive[0] |= (msg.id == 9);
      receive[1] |= (msg.id == 10);
      receive[2] |= (msg.id == 15);
    }
    printf("\nwaiting CAN %2d %2d %2d", 9 * !receive[0], 10 * !receive[1], 15 * !receive[2]);
    ThisThread::sleep_for(5ms);
  }
}
