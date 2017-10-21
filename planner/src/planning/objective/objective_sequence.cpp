#include <dmp/planning/objective/objective_sequence.h>

namespace dmp
{
ObjectiveSequence::ObjectiveSequence(std::initializer_list<std::shared_ptr<Objective>> list)
    : sequence_(list)
{
}

void ObjectiveSequence::addObjective(const std::shared_ptr<Objective>& objective)
{
  sequence_.push_back(objective);
}

const std::vector<std::shared_ptr<Objective>>& ObjectiveSequence::getSequence() const noexcept
{
  return sequence_;
}
}
