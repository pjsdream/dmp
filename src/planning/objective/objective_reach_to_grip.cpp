#include <dmp/planning/objective/objective_reach_to_grip.h>
#include <dmp/planning/environment/interactable_object.h>

namespace dmp
{
ObjectiveReachToGrip::ObjectiveReachToGrip(const std::shared_ptr<InteractableObject>& object)
    : object_(object)
{
}
}
