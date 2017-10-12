#include <dmp/planning/cost/cost_smoothness.h>
#include <include/dmp/robot/robot_configuration.h>

namespace dmp
{
CostSmoothness::CostSmoothness(double weight)
    : Cost(weight)
{
}

std::tuple<double,
           Eigen::VectorXd,
           Eigen::VectorXd> CostSmoothness::computeCost(const RobotConfiguration& configuration)
{
  Eigen::VectorXd zeroes(configuration.getPositions().size());
  zeroes.setZero();

  const auto& velocities = configuration.getVelocities();

  return std::make_tuple(velocities.squaredNorm() * getWeight(), zeroes, 2. * getWeight() * velocities);
}
}
