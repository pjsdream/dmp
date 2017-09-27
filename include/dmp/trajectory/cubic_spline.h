#ifndef DMP_CUBIC_SPLINE_H
#define DMP_CUBIC_SPLINE_H

#include <vector>

namespace dmp
{
class CubicSpline
{
public:
  CubicSpline() = delete;
  explicit CubicSpline(double t, int num_curves);

  double position(double t) const;
  double velocity(double t) const;

private:
  double position(int i, double u) const;
  double velocity(int i, double u) const;

  double t_;
  double dt_;
  std::vector<double> p_;
  std::vector<double> v_;
};
}

#endif //DMP_CUBIC_SPLINE_H
