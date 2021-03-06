#include <dmp/utils/rate.h>
#include <thread>

namespace dmp
{
Rate Rate::withDuration(double duration)
{
  return Rate(1. / duration);
}

Rate::Rate(double rate)
    : duration_(1. / rate), count_(0)
{
  start_time_ = std::chrono::high_resolution_clock::now();
}

void Rate::reset() noexcept
{
  count_ = 0;
}

double Rate::remainingTime() const
{
  double wait_until = (count_ + 1) * duration_;
  std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time_;
  return wait_until - diff.count();
}

double Rate::duration() const
{
  return duration_;
}

void Rate::sleep()
{
  count_++;

  double wait_until = count_ * duration_;
  std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time_;
  auto sleep_for = wait_until - diff.count();

  if (sleep_for > 0)
    std::this_thread::sleep_for(std::chrono::duration<double>(sleep_for));
}
}
