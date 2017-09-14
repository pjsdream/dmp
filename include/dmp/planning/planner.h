#ifndef DMP_PLANNER_H
#define DMP_PLANNER_H

#include <memory>

namespace dmp
{
class PlanningOption;
class Planner
{
public:
  Planner() = delete;
  explicit Planner(const PlanningOption& option);
  ~Planner();

  Planner(const Planner& rhs) = delete;
  Planner& operator = (const Planner& rhs) = delete;

  Planner(Planner&& rhs) = default;
  Planner& operator = (Planner&& rhs) = default;

  void plan();

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif //DMP_PLANNER_H
