#ifndef DMP_OBJECTIVE_REACH_GRIP_H
#define DMP_OBJECTIVE_REACH_GRIP_H

#include "objective.h"
#include <dmp/planning/environment/interactable_object.h>

namespace dmp
{
class ObjectiveReachToGrip : public Objective
{
public:
  ObjectiveReachToGrip() = delete;
  explicit ObjectiveReachToGrip(const std::shared_ptr<InteractableObject>& object);
  ~ObjectiveReachToGrip() override = default;

private:
  std::shared_ptr<InteractableObject> object_;
};
}

#endif //DMP_OBJECTIVE_REACH_GRIP_H
