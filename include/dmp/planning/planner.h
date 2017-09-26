#ifndef DMP_PLANNER_H
#define DMP_PLANNER_H

#include <memory>

#include <dmp/comm/node.h>
#include <dmp/comm/publisher.h>
#include <dmp/comm/subscriber.h>
#include <dmp/rendering/request/request.h>
#include <dmp/trajectory/trajectory.h>
#include <dmp/robot/robot_state.h>

namespace dmp
{
class PlanningOption;
class RobotModel;
class Motion;
class Environment;
class Shape;

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

  Subscriber<RobotState>& getRobotStateSubscriber();
  Publisher<Request>& getRendererPublisher();
  Publisher<Trajectory>& getTrajectoryPublisher();

protected:
  void run() override;

private:
  void drawGround();
  void drawRobotModel();
  void drawEnvironment();

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setMotion(const std::shared_ptr<Motion>& motion);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

  Subscriber<RobotState> robot_state_subscriber_;
  Publisher<Request> renderer_publisher_;
  Publisher<Trajectory> trajectory_publisher_;

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;

  std::vector<std::vector<std::shared_ptr<Shape>>> bounding_volumes_;
};
}

#endif //DMP_PLANNER_H
