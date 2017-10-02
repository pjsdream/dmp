#include <dmp/planning/cost/cost_smoothness.h>
#include <dmp/planning/robot_configuration.h>

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

  return std::make_tuple(0., zeroes, zeroes);
}
}
