#ifndef DMP_TIMER_H
#define DMP_TIMER_H

#include <chrono>

namespace dmp
{
class Timer
{
public:
  Timer() = delete;
  explicit Timer(double time);

  bool isOver();

private:
  double time_;
  bool already_over_;
  std::chrono::high_resolution_clock::time_point start_time_;
};
}

#endif //DMP_TIMER_H
