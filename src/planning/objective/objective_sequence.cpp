#include <dmp/planning/objective/objective_sequence.h>

namespace dmp
{
void ObjectiveSequence::addObjective(const std::shared_ptr<Objective>& objective)
{
  sequence_.push_back(objective);
}

const std::vector<std::shared_ptr<Objective>>& ObjectiveSequence::getSequence() const noexcept
{
  return sequence_;
}
}
