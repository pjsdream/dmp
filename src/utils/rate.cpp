#include <dmp/utils/rate.h>
#include <thread>

namespace dmp
{
Rate::Rate(int rate)
    : duration_(1. / rate), count_(0)
{
  start_time_ = std::chrono::high_resolution_clock::now();
}

void Rate::reset() noexcept
{
  count_ = 0;
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
