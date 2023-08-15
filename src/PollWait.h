#ifndef RCT_POLL_WAIT_H_
#define RCT_POLL_WAIT_H_

#include <chrono>

template<class Clock = std::chrono::steady_clock>
struct PollWait {
  PollWait() : pre_{Clock::now()} {}
  auto operator()(typename Clock::duration wait) {
    auto now = Clock::now();
    auto delta = now - pre_;
    auto elapsed = delta > wait;
    if(elapsed) pre_ = now;
    return Result{delta, elapsed};
  }
 private:
  struct Result : Clock::duration {
    operator bool() const {
      return elapsed;
    }
    bool elapsed;
  };
  typename Clock::time_point pre_;
};

#endif  // RCT_POLL_WAIT_H_
