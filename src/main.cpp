#include <ChassisPid.h>
#include <Pid.h>
#include <SteerDrive.h>
#include <mbed.h>

#include <algorithm>
#include <complex>
#include <utility>

#include "C620.h"
#include "PollWait.h"
#include "Rs485.h"
#include "SendCrtp.h"
#include "SensorBoard.h"
#include "protocol.h"

// const variable
constexpr auto dc_id = 5;
constexpr int enc_rot = 12934;   // steer unit 1周あたりエンコーダが読むパルス数
constexpr int one_rotate = 363;  // steer 1周あたりのオドメトリ
constexpr int one_meter = 110;   // steer 1mあたりのオドメトリ

// prototype
/// @brief センサ基板とコントローラの入力を待機する
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
struct Amt21 {
  static constexpr int rotate = 4096;

  uint8_t address;
  int32_t pos;
  uint16_t pre_pos;

  bool request_pos() {
    rs485.uart_transmit({address});
    if(uint16_t now_pos; rs485.uart_receive(&now_pos, sizeof(now_pos), 10ms) && is_valid(now_pos)) {
      now_pos = (now_pos & 0x3fff) >> 2;
      int16_t diff = now_pos - pre_pos;
      uint16_t sign = (diff & 0x800) * 0x1e;  // 12-bit目を符号ビットとして扱う
      pos += sign | (diff & 0xfff);
      pre_pos = now_pos;
      return true;
    }
    return false;
  }
  void request_reset() {
    rs485.uart_transmit({uint8_t(address + 2), 0x75});
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
} amt[] = {{0x50}, {0x54}, {0x58}, {0x5C}};
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
  auto get() const& {
    auto pos = pos_;
    pos.x_milli /= one_meter;
    pos.y_milli /= one_meter;
    pos.ang_rad *= -2 * M_PI / one_rotate;
    return pos;
  }
 private:
  rct::Coordinate pos_;
};
constexpr rct::PidGain drive_gain{1.2, 0.3};
constexpr rct::PidGain gain{3.65, 0.85, 0.0005};
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
SteerOdom odom{};
SteerUnit unit[4] = {};
auto f = [](std::array<std::complex<float>, 4> cmp) {
  for(int i = 0; i < 4; ++i) {
    int new_tag_pos = enc_rot / 2 / M_PI * arg(cmp[i]);
    int offset = new_tag_pos - (-amt[i].pos);
    int r = std::round(2.0 * offset / enc_rot);
    int drive_dir = 2 * (r % 2 == 0) - 1;
    unit[i].target_rpm = abs(cmp[i]) * 9000 * drive_dir;  // max 9000rpm
    new_tag_pos = new_tag_pos - r * (enc_rot / 2);
    if(cmp[i] != 0.0f) {
      unit[i].target_pos = new_tag_pos;
    } else {
      auto diff = new_tag_pos - unit[i].target_pos;
      unit[i].target_pos += 0.5 * diff;
    }
  }
};
rct::ChassisPid<rct::SteerDrive<4>> steer{f, {0.5, 1.2}};
#if 0
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
#else
struct Controller {
  rct::Velocity vel = {};

  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.id == Command::ID && msg.len == 8) {
      Command cmd;
      memcpy(&cmd, msg.data, sizeof(Command));
      switch(cmd.tag) {
        case Command::Tag::SET_TARGET_VELOCITY: {
          vel = {cmd.set_target_velocity.vx * 1e-3f, cmd.set_target_velocity.vy * 1e-3f,
                 cmd.set_target_velocity.ang_vel * 1e-3f};
          if(std::hypot(vel.x_milli, vel.y_milli) < 0.1) {
            vel.x_milli = 0;
            vel.y_milli = 0;
          }
          if(std::abs(vel.ang_rad) < 0.1) vel.ang_rad = 0;
        } break;
        default:
          break;
      }
    }
  }
  rct::Velocity get_vel() const {
    return vel;
  }
} controller;
#endif

int main() {
  // put your setup code here, to run once:
  printf("\nsetup\n");
  wait_can();

  timer.start();
  while(1) {
    auto now = timer.elapsed_time();
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

    // アブソ読み取り
    for(auto& e: amt) {
      e.request_pos();
    }

    // 10msごとにCAN送信
    if(static PollWait<Kernel::Clock> wait{}; auto delta = wait(10ms)) {
      rct::Velocity vel = controller.get_vel();

      // C620が生存なら
      if(now - pre_alive < 100ms) {
        // Odom で自己位置を推定
        std::complex<float> diff[4];
        for(auto i = 0; i < 4; ++i) {
          float rho = dji.data[i].rpm * delta.count() * 1e-5;
          float theta = 2 * M_PI / enc_rot * amt[i].pos;
          diff[i] = std::polar(rho, theta);
        }
        odom.integrate(diff);
      }

      // 変位より速度を計算
      static auto pre_coo = rct::Coordinate{};
      auto now_coo = odom.get();
      auto now_vel = -1 * (now_coo - pre_coo) / delta;
      float est_vel_rad = (now_coo - pre_coo).ang_rad * 3 / 4;
      pre_coo = now_coo;

      // ±1の範囲にスケール
      now_vel.x_milli *= -127.0 / 285;
      now_vel.y_milli *= 127.0 / 285;
      now_vel.ang_rad *= 96.0 / 431;

      // 姿勢角 = 現在の角度 + 予測変位
      float offset_rad = odom.get().ang_rad + M_PI / 2 + est_vel_rad;
      if(vel != rct::Velocity{}) {
        steer.pid_move(vel, now_vel, delta, offset_rad);
      } else {
        steer.move(vel, offset_rad);
        steer.refresh();
      }

      for(auto i = 0; i < 4; ++i) {
        if(now - pre_alive < 100ms || true) {
          // pidの計算
          // TODO encの更新に合わせる？ delta timeを
          std::tie(dji.pwm[i], dc_sender.pwm[i]) = unit[i].calc_pid(dji.data[i].rpm, -amt[i].pos, delta);
        } else {
          // fail時, 出力を1/2倍していく
          dji.pwm[i] /= 2;
          dc_sender.pwm[i] /= 2;
          // I値を消す
          unit[i].pid_drive.refresh();
          unit[i].pid_steer.refresh();
        }
      }

      // printf("enc:");
      // for(auto i = 0; i < 4; ++i) {
      //   printf("% 4d\t", amt[i].pre_pos);
      // }
      printf("pos:");
      for(auto i = 0; i < 4; ++i) {
        printf("% 6ld ", -amt[i].pos);
      }

      printf("now_vel:");
      printf("% 4d ", (int)(now_vel.x_milli * 1e2));
      printf("% 4d ", (int)(now_vel.y_milli * 1e2));
      printf("% 4d ", (int)(now_vel.ang_rad * 1e2));

      printf("vel:");
      printf("% 4d ", (int)(vel.x_milli * 1e2));
      printf("% 4d ", (int)(vel.y_milli * 1e2));
      printf("% 4d ", (int)(vel.ang_rad * 1e2));

      // printf("tag:");
      // for(auto& e: unit) {
      //   printf("% 5d\t", e.target_pos);
      // }
      // printf("rpm:");
      // for(int i = 0; i < 4; ++i) {
      //   printf("% 5d\t", reader.data[i].rpm);
      // }
      // printf("tag:");
      // for(auto& e: unit) {
      //   printf("% 5d\t", e.target_rpm);
      // }

      printf("est:");
      printf("% 7d ", int(odom.get().x_milli * 1e3));
      printf("% 7d ", int(odom.get().y_milli * 1e3));
      printf("% 5d ", int(odom.get().ang_rad * 1e3));

      printf(" dc:");
      for(auto& e: dc_sender.pwm) {
        printf("% 5d ", e);
      }
      printf(" ac:");
      for(int i = 0; i < 4; ++i) {
        printf("% 5d ", dji.pwm[i]);
      }

      printf("\n");

      dc_sender.send();
      dji.send();
    }
  }
}

void wait_can() {
  bool receive[3] = {1, 1, 0};
  while(!(receive[0] && receive[1] && receive[2])) {
    if(can1.read(msg)) {
      sensor_board.read(msg);
      controller.read(msg);
      receive[0] |= (msg.id == 9);
      receive[1] |= (msg.id == 10);
      receive[2] |= (msg.id == 15 || msg.id == 200);
    }
    printf("\nwaiting CAN %2d %2d %2d", 9 * !receive[0], 10 * !receive[1], 15 * !receive[2]);
    ThisThread::sleep_for(5ms);
  }
}
