#include <dmp/planning/cost/cost.h>
#include <include/dmp/robot/robot_configuration.h>

namespace dmp
{
Cost::Cost(double weight)
    : weight_(weight)
{
}

double Cost::getWeight() const noexcept
{
  return weight_;
}

std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> Cost::computeCost(const RobotConfiguration& configuration)
{
  Eigen::VectorXd zeroes(configuration.getPositions().size());
  zeroes.setZero();

  return std::make_tuple(0., zeroes, zeroes);
};
}
