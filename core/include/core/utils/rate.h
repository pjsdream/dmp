#ifndef DMP_RATE_H
#define DMP_RATE_H

#include <chrono>

namespace dmp
{
class Rate
{
public:
  Rate() = delete;
  explicit Rate(double rate);

  static Rate withDuration(double duration);

  void reset() noexcept;

  double remainingTime() const;
  double duration() const;

  void sleep();

private:
  const double duration_;
  int count_;
  std::chrono::high_resolution_clock::time_point start_time_;
};
}

#endif //DMP_RATE_H
