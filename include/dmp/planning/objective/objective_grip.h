#ifndef DMP_OBJECTIVE_GRIP_H
#define DMP_OBJECTIVE_GRIP_H

#include "objective.h"

#include <memory>

namespace dmp
{
class InteractableObject;

class ObjectiveGrip : public Objective
{
public:
  ObjectiveGrip() = delete;
  explicit ObjectiveGrip(const std::shared_ptr<InteractableObject>& object);
  ~ObjectiveGrip() override = default;

  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeCost(const RobotConfiguration& configuration) override;

private:
  std::shared_ptr<InteractableObject> object_;
};
}

#endif //DMP_OBJECTIVE_GRIP_H
