//
// Created by Kurosu Chan on 2023/5/22.
//

#ifndef SIMPLE_INSTANT_H
#define SIMPLE_INSTANT_H

#include <etl/delegate.h>
#include <chrono>

class Instant {
  decltype(esp_timer_get_time()) time;

public:
  Instant() {
    this->time = esp_timer_get_time();
  }

  auto elapsed() {
    auto now      = esp_timer_get_time();
    auto diff     = now - this->time;
    auto duration = std::chrono::duration<int64_t, std::micro>(diff);
    return duration;
  }

  void reset() {
    auto now   = esp_timer_get_time();
    this->time = now;
  }

  auto elapsed_and_reset() {
    auto now      = esp_timer_get_time();
    auto duration = now - this->time;
    this->time    = now;
    return duration;
  }

  [[nodiscard]] auto count() const {
    return time;
  }

  template<typename T>
  bool try_run(etl::delegate<T> f, std::chrono::duration<uint64_t, std::milli> d) {
    if (this->elapsed() > d) {
      f();
      this->reset();
      return true;
    } else {
      return false;
    }
  }
};

#endif // SIMPLE_INSTANT_H
