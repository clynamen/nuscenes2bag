#pragma once 

#include <chrono>

namespace nuscenes2bag {

template <typename T> class RunEvery {
public:
  RunEvery(std::chrono::milliseconds periodMs, T &&lambda)
      : periodMs(periodMs), lastExecutionTime(std::chrono::system_clock::now()),
        lambda(lambda) {}

  void update() {
    auto now = std::chrono::system_clock::now();
    auto dtMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - lastExecutionTime);
    if (dtMs > periodMs) {
      lambda();
      lastExecutionTime = now;
    }
  }

  std::chrono::time_point<std::chrono::system_clock> lastExecutionTime;
  std::chrono::milliseconds periodMs;
  T lambda;
};

}