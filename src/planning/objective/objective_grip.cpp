#include <dmp/planning/objective/objective_grip.h>

namespace dmp
{
ObjectiveGrip::ObjectiveGrip(const std::shared_ptr<InteractableObject>& object)
    : object_(object)
{
}
}
