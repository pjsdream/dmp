#include <dmp/planning/objective/objective_grip.h>

namespace dmp
{
ObjectiveGrip::ObjectiveGrip(const std::shared_ptr<InteractableObject>& object)
    : object_(object)
{
}

std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> ObjectiveGrip::computeCost(const RobotConfiguration& configuration)
{
  return Objective::computeCost(configuration);
}
}
