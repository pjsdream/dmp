#ifndef DMP_RATE_H
#define DMP_RATE_H

#include <chrono>

namespace dmp
{
class Rate
{
public:
  Rate() = delete;
  explicit Rate(int rate);

  void reset() noexcept;

  double remainingTime();

  void sleep();

private:
  const double duration_;
  int count_;
  std::chrono::high_resolution_clock::time_point start_time_;
};
}

#endif //DMP_RATE_H
