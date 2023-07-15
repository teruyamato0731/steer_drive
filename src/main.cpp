#include <Odom.h>
#include <Pid.h>
#include <SteerDrive.h>
#include <mbed.h>

#include <algorithm>

#include "SensorBoard.h"
#include "swap_endian.h"

// const variable
constexpr auto dc_id = 5;
constexpr auto enc_rot = 1600;

// prototype
void wait_can();
void calibration();
void set_zero_pos(int);

// IO
BufferedSerial pc{USBTX, USBRX, 115200};
CAN can1{PA_11, PA_12, (int)1e6};
CAN can2{PB_12, PB_13, (int)1e6};
CANMessage msg;
Timer timer;
InterruptIn p_i[4] = {{PC_4}, {PC_5}, {PC_6}, {PC_7}};

// struct definition
/// @tparam can global変数のみを受け取る
/// @ref https://marycore.jp/prog/cpp/non-type-template-parameter/
struct C620Sender {
  static constexpr int max = 16384;
  int16_t pwm[8];

  CANMessage fw() {
    int16_t data[4];
    data[0] = swap_endian(pwm[0]);
    data[1] = swap_endian(pwm[1]);
    data[2] = swap_endian(pwm[2]);
    data[3] = swap_endian(pwm[3]);
    return CANMessage{0x200, (uint8_t*)data, 8};
  }
  CANMessage bw() {
    int16_t data[4];
    data[0] = swap_endian(pwm[4]);
    data[1] = swap_endian(pwm[5]);
    data[2] = swap_endian(pwm[6]);
    data[3] = swap_endian(pwm[7]);
    return CANMessage{0x1FF, (uint8_t*)data, 8};
  }
  bool send() {
    return can2.write(fw()) && can2.write(bw());
  }
};
// c620から受け取るデータ
struct C620Reader {
  // index operator
  void read(const CANMessage& msg) {
    if(msg.format == CANStandard && msg.type == CANData && msg.len == 8 && 0x200 <= msg.id && msg.id <= 0x208) {
      data[msg.id - 0x201u].set(msg.data);
    }
  }
  //C620ReadVal
  struct {
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
  } data[8];
};
struct DCSender {
  static constexpr int max = INT16_MAX * 0.7;
  int16_t pwm[4];
  void send() {
    can1.write(CANMessage{dc_id, (uint8_t*)pwm, 8});
  }
};
constexpr rct::PidGain drive_gain{0.8f, 0.5f};
constexpr rct::PidGain gain{15, 0.75};
struct SteerUnit {
  auto calc_pid(const int rpm, const int pos, const std::chrono::microseconds& delta_time) {
    auto drive = pid_drive.calc(target_rpm, rpm, delta_time);
    drive = std::clamp(drive, -0.7f * C620Sender::max, 0.7f * C620Sender::max);
    auto steer = pid_steer.calc(target_pos, pos, delta_time);
    steer = -std::clamp(steer, -0.7f * DCSender::max, 0.7f * DCSender::max);
    return std::tuple{drive, steer};
  }
  rct::Pid<float> pid_drive = {drive_gain};
  rct::Pid<float> pid_steer = {gain};
  int zero_pos;
  int target_rpm;
  int target_pos;
};

// Control
SensorBoard sensor_board{9u, 10u};
C620Reader reader{};
C620Sender sender{};
DCSender dc_sender{};
rct::Odom<4> odom{};
SteerUnit unit[4] = {};
rct::SteerDrive<4> steer{[](std::array<std::complex<float>, 4> cmp) {
  for(int i = 0; i < 4; ++i) {
    int pos = sensor_board.enc[i] - unit[i].zero_pos;
    int new_tag_pos = enc_rot / 2 / M_PI * arg(cmp[i]);
    int offset = new_tag_pos - pos;
    int r = std::round(2.0 * offset / enc_rot);
    int drive_dir = 1 - 2 * (r % 2);
    unit[i].target_rpm = abs(cmp[i]) * 6000 * drive_dir;  // max 9000rpm
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
} controller;

int main() {
  // put your setup code here, to run once:
  wait_can();
  calibration();

  printf("\nsetup\n");

  for(auto i = 0; i < 4; ++i) {
    p_i[i].disable_irq();
  }

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
      reader.read(msg);
      if(0x200u < msg.id && msg.id <= 0x208u) {
        // C620の生存CK
        pre_alive = now;
      }
    }

    // C620が生存なら
    if(now - pre_alive < 100ms) {
      // Odom で自己位置を推定
      int diff[4];
      for(auto i = 0; i < 4; ++i) {
        diff[i] = reader.data[i].rpm;
      }
      odom.integrate(diff);
    }

    // 10msごとにCAN送信
    if(now - pre > 10ms) {
      rct::Velocity vel = {controller.stick[0] / 128.0f, controller.stick[1] / 128.0f, controller.stick[2] / 128.0f};

      printf("vel:");
      printf("%3d\t", (int)(vel.x_milli * 128));
      printf("%3d\t", (int)(vel.y_milli * 128));
      printf("%3d\t", (int)(vel.ang_rad * 128));

      steer.move(vel);
      for(auto i = 0; i < 4; ++i) {
        if(now - pre_alive < 100ms || true) {
          // pidの計算
          // reader[0]~[3] を使用するため djiモータのIDを1~4にしておく
          // sensor_board の 0~3に接続
          int pos = sensor_board.enc[i] - unit[i].zero_pos;
          // TODO encの更新に合わせる？ delta timeを
          std::tie(sender.pwm[i], dc_sender.pwm[i]) = unit[i].calc_pid(reader.data[i].rpm, pos, now - pre);
        } else {
          // fail時, 出力を1/2倍していく
          sender.pwm[i] /= 2;
          dc_sender.pwm[i] /= 2;
          // I値を消す
          unit[i].pid_drive.refresh();
          unit[i].pid_steer.refresh();
        }
      }
      // printf("rpm:");
      // for(auto& e: reader.data) {
      //   printf("% 4d\t", e.rpm);
      // }
      // printf("enc:");
      // for(auto e: sensor_board.enc) {
      //   printf("% 5d\t", e);
      // }
      // printf("zero:");
      // for(auto i = 0; i < 4; ++i) {
      //   printf("% 5d\t", unit[i].zero_pos);
      // }
      printf("pos:");
      for(auto i = 0; i < 4; ++i) {
        printf("% 4d\t", sensor_board.enc[i] - unit[i].zero_pos);
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
      printf("\n");

      dc_sender.send();
      sender.send();

      pre = now;
    }
  }
}

void set_zero_pos(const int i) {
  constexpr int offset[4] = {-enc_rot * 85 / 360, enc_rot * 80 / 360, -enc_rot * 80 / 360, enc_rot * 80 / 360};
  unit[i].zero_pos = (sensor_board.enc[i] - offset[i]) % enc_rot;
}

void wait_can() {
  bool receive[3] = {};
  while(!(receive[0] && receive[1] && receive[2])) {
    if(can1.read(msg)) {
      receive[0] |= (msg.id == 9);
      receive[1] |= (msg.id == 10);
      receive[2] |= (msg.id == 15);
    }
    printf("\nwaiting CAN %2d %2d %2d", 9 * !receive[0], 10 * !receive[1], 15 * !receive[2]);
    ThisThread::sleep_for(5ms);
  }
}

void calibration() {
  volatile bool stop[4] = {};
  for(auto i = 0; i < 4; ++i) {
    p_i[i].fall([&, i] {
      stop[i] = true;
      set_zero_pos(i);
    });
  }
  while(std::any_of(begin(stop), end(stop), [](bool x) {
    return !x;
  })) {
    if(can1.read(msg)) {
      sensor_board.read(msg);
      controller.read(msg);
    }

    for(int i = 0; i < 4; ++i) {
      dc_sender.pwm[i] = 0.10 * dc_sender.max * !stop[i];
    }

    printf("\ncalibrating ");
    for(auto& e: p_i) {
      printf("%d", e.read());
    }
    printf(" ");
    for(auto& e: dc_sender.pwm) {
      printf("%4d\t", e);
    }
    for(auto e: sensor_board.enc) {
      printf("% 5d\t", e);
    }

    rct::Velocity vel = {controller.stick[1] / 128.0f, controller.stick[0] / 128.0f, controller.stick[2] / 128.0f};
    printf("vel:");
    printf("%3d\t", (int)(vel.x_milli * 128));
    printf("%3d\t", (int)(vel.y_milli * 128));
    printf("%3d\t", (int)(vel.ang_rad * 128));

    dc_sender.send();
    ThisThread::sleep_for(10ms);
  }

  std::fill(begin(dc_sender.pwm), end(dc_sender.pwm), 0);
  dc_sender.send();
}
