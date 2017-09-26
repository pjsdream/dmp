#ifndef DMP_OBJECTIVE_REACH_TO_PLACE_H
#define DMP_OBJECTIVE_REACH_TO_PLACE_H

#include "objective.h"

#include <memory>

namespace dmp
{
class InteractableObject;

class ObjectiveReachToPlace : public Objective
{
public:
  ObjectiveReachToPlace() = delete;
  explicit ObjectiveReachToPlace(const std::shared_ptr<InteractableObject>& object);
  ~ObjectiveReachToPlace() override = default;

private:
  std::shared_ptr<InteractableObject> object_;
};
}

#endif //DMP_OBJECTIVE_REACH_TO_PLACE_H
