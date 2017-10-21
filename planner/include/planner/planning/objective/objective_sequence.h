#ifndef DMP_OBJECTIVE_SEQUENCE_H
#define DMP_OBJECTIVE_SEQUENCE_H

#include <vector>
#include <memory>

namespace dmp
{
class Objective;

class ObjectiveSequence
{
public:
  ObjectiveSequence() = default;
  ObjectiveSequence(std::initializer_list<std::shared_ptr<Objective>> list);

  void addObjective(const std::shared_ptr<Objective>& objective);

  const std::vector<std::shared_ptr<Objective>>& getSequence() const noexcept;

private:
  std::vector<std::shared_ptr<Objective>> sequence_;
};
}

#endif //DMP_OBJECTIVE_SEQUENCE_H
