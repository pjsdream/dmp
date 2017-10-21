#ifndef DMP_STOPWATCH_H
#define DMP_STOPWATCH_H

#include <chrono>

namespace dmp
{
class Stopwatch
{
public:
  Stopwatch();

  double time();

private:
  std::chrono::high_resolution_clock::time_point start_time_;
};
}
#endif //DMP_STOPWATCH_H
