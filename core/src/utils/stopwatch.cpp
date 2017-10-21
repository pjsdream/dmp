#include <dmp/utils/stopwatch.h>

namespace dmp
{
Stopwatch::Stopwatch()
    : start_time_(std::chrono::high_resolution_clock::now())
{
}

double Stopwatch::time()
{
  std::chrono::duration<double> diff = std::chrono::high_resolution_clock::now() - start_time_;
  return diff.count();
}
}
