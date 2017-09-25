#ifndef DMP_PLANNER_H
#define DMP_PLANNER_H

#include <memory>

#include <dmp/comm/node.h>
#include <dmp/comm/publisher.h>
#include <dmp/rendering/request/request.h>
#include <dmp/trajectory/trajectory.h>

namespace dmp
{
class PlanningOption;

class Planner : public Node
{
public:
  Planner() = delete;
  explicit Planner(const PlanningOption& option);
  ~Planner() override;

  Planner(const Planner& rhs) = delete;
  Planner& operator = (const Planner& rhs) = delete;

  Planner(Planner&& rhs) = delete;
  Planner& operator = (Planner&& rhs) = delete;

  Publisher<Request>& getRendererPublisher();
  Publisher<Trajectory>& getTrajectoryPublisher();

protected:
  void run() override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif //DMP_PLANNER_H
