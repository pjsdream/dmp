#ifndef DMP_OBJECTIVE_REACH_GRIP_H
#define DMP_OBJECTIVE_REACH_GRIP_H

#include "objective.h"

#include <memory>

namespace dmp
{
class InteractableObject;

class ObjectiveReachToGrip : public Objective
{
public:
  ObjectiveReachToGrip() = delete;
  explicit ObjectiveReachToGrip(const std::shared_ptr<InteractableObject>& object);
  ~ObjectiveReachToGrip() override = default;

  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeCost(const RobotConfiguration& configuration) override;

private:
  std::shared_ptr<InteractableObject> object_;
};
}

#endif //DMP_OBJECTIVE_REACH_GRIP_H
