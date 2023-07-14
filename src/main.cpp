#include <mbed.h>

BufferedSerial pc{USBTX, USBRX, 115200};

// CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};
// CANMessage msg;

Timer timer;

int main() {
  printf("\nsetup\n");
  timer.start();
  auto pre = timer.elapsed_time();
  while(1) {
    auto now = timer.elapsed_time();
    if(now - pre > 20ms) {
      printf("hoge\n");
      pre = now;
    }
  }
}
