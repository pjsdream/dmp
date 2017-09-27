#include <dmp/trajectory/cubic_spline.h>

namespace dmp
{
CubicSpline::CubicSpline(double t, int num_curves)
    : dt_(t / num_curves)
{
  p_.resize(num_curves + 1);
  v_.resize(num_curves + 1);
}

double CubicSpline::position(double t) const
{
  auto i = static_cast<int>(t / dt_);
  if (t == t_)
    i = static_cast<int>(p_.size() - 2);
  auto u = (t - dt_ * i) / dt_;

  return position(i, u);
}

double CubicSpline::velocity(double t) const
{
  auto i = static_cast<int>(t / dt_);
  if (t == t_)
    i = static_cast<int>(p_.size() - 2);
  const auto u = (t - dt_ * i) / dt_;

  return velocity(i, u);
}

double CubicSpline::position(int i, double u) const
{
  const auto u2 = u * u;
  const auto u3 = u2 * u;

  return (2. * u3 - 3. * u2 + 1.) * p_[i] + (u3 - 2. * u2 + u) * v_[i] + (-2. * u3 + 3. * u2) * p_[i + 1]
      + (u3 - u2) * v_[i + 1];
}

double CubicSpline::velocity(int i, double u) const
{
  const auto u2 = u * u;

  return (6. * u2 - 6. * u) * p_[i] + (3. * u2 - 4. * u + 1.) * v_[i] + (-6. * u2 + 6. * u) * p_[i + 1]
      + (3. * u2 - 2. * u) * v_[i + 1];
}
}
