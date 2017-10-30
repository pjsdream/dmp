#include <core/utils/timer.h>

#include <thread>

namespace dmp
{
Timer::Timer(double time)
    : start_time_(std::chrono::high_resolution_clock::now()), time_(time), already_over_(false)
{
}

bool Timer::isOver()
{
  if (already_over_)
    return true;

  std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time_;
  if (diff.count() >= time_)
  {
    already_over_ = true;
    return true;
  }
  return false;
}

void Timer::sleepUntil()
{
  if (!isOver())
  {
    std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time_;
    std::this_thread::sleep_for(diff);
  }
}
}
