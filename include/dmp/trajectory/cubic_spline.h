#ifndef DMP_CUBIC_SPLINE_H
#define DMP_CUBIC_SPLINE_H

#include <vector>

#include <Eigen/Dense>

namespace dmp
{
class CubicSpline
{
public:
  CubicSpline() = delete;
  explicit CubicSpline(int num_curves);

  double position(double t) const;
  double velocity(double t) const;

  double& controlPosition(int i);
  double& controlVelocity(int i);

  void fitting(const std::vector<std::tuple<double, double>>& samples, double p0, double v0);

  int getCurveIndex(double t) const noexcept;
  Eigen::Vector4d getPositionCoefficients(double t) const noexcept;
  Eigen::Vector4d getVelocityCoefficients(double t) const noexcept;

private:
  double position(int i, double u) const;
  double velocity(int i, double u) const;

  std::vector<double> p_;
  std::vector<double> v_;
};
}

#endif //DMP_CUBIC_SPLINE_H
