#ifndef DMP_CUBIC_SPLINE_H
#define DMP_CUBIC_SPLINE_H

#include <vector>

namespace dmp
{
class CubicSpline
{
public:
  CubicSpline() = delete;
  CubicSpline(double t, int num_curves);

  double getT() const;

  double position(double t) const;
  double velocity(double t) const;

  double& controlPosition(int i);
  double& controlVelocity(int i);

  void fitting(const std::vector<std::tuple<double, double>>& samples, double p0, double v0);

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