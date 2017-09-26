#include <dmp/planning/objective/objective_reach_to_place.h>

namespace dmp
{
ObjectiveReachToPlace::ObjectiveReachToPlace(const std::shared_ptr<InteractableObject>& object)
    : object_(object)
{
}
}
