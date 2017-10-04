#include <dmp/trajectory/cubic_spline.h>
#include <Eigen/Dense>

namespace dmp
{
CubicSpline::CubicSpline(int num_curves)
{
  p_.resize(num_curves + 1);
  v_.resize(num_curves + 1);
}

double CubicSpline::position(double t) const
{
  auto i = static_cast<int>(t);
  if (i == p_.size() - 1)
    i = static_cast<int>(p_.size() - 2);
  auto u = t - i;

  return position(i, u);
}

double CubicSpline::velocity(double t) const
{
  auto i = static_cast<int>(t);
  if (i == p_.size() - 1)
    i = static_cast<int>(p_.size() - 2);
  const auto u = t - i;

  return velocity(i, u);
}

double& CubicSpline::controlPosition(int i)
{
  return p_[i];
}

double& CubicSpline::controlVelocity(int i)
{
  return v_[i];
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

void CubicSpline::fitting(const std::vector<std::tuple<double, double>>& samples, double p0, double v0)
{
  // Construct least square problem
  Eigen::MatrixXd A(samples.size(), p_.size() * 2);
  Eigen::VectorXd b(samples.size());
  A.setZero();

  int row = 0;
  for (const auto& tuple : samples)
  {
    const auto time = std::get<0>(tuple);
    const auto position = std::get<1>(tuple);

    auto col = static_cast<int>(time);
    auto u = time - col;
    if (std::abs(time - (p_.size() - 1)) < 1e-5)
    {
      col--;
      u = 1.;
    }
    col *= 2;

    // Skipping the first position/velocity variables, because these are fixed with p0/v0.
    col -= 2;

    const auto u2 = u * u;
    const auto u3 = u2 * u;

    // Position sample
    b(row) = position;
    if (col >= 0)
    {
      A(row, col) = 2. * u3 - 3. * u2 + 1.;
      A(row, col + 1) = u3 - 2. * u2 + u;
    }
    else
    {
      b(row) -= (2. * u3 - 3. * u2 + 1.) * p0 + (u3 - 2. * u2 + u) * v0;
    }

    A(row, col + 2) = -2. * u3 + 3. * u2;
    A(row, col + 3) = u3 - u2;

    row++;
  }

  // Least square using SVD decomposition
  auto x = static_cast<Eigen::VectorXd>(A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b));

  // Assign control points from the least square solution
  p_[0] = p0;
  v_[0] = v0;
  for (int i = 1; i < p_.size(); i++)
  {
    p_[i] = x(2 * i - 2);
    v_[i] = x(2 * i - 1);
  }
}
}
