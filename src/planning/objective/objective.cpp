#include <dmp/planning/objective/objective.h>
#include <dmp/planning/robot_configuration.h>

namespace dmp
{
std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> Objective::computeCost(const RobotConfiguration& configuration)
{
  Eigen::VectorXd zeroes(configuration.getPositions().rows());
  zeroes.setZero();

  return std::make_tuple(0., zeroes, zeroes);
}
}
