#ifndef DMP_PLANNER_H
#define DMP_PLANNER_H

#include <memory>

#include <dmp/comm/node.h>
#include <dmp/comm/publisher.h>
#include <dmp/comm/subscriber.h>
#include <dmp/rendering/request/request.h>
#include <dmp/trajectory/trajectory.h>
#include <dmp/robot/robot_state.h>
#include <dmp/planning/objective/objective.h>

namespace dmp
{
class PlanningOption;
class RobotModel;
class Motion;
class Environment;
class Shape;
class CubicSplineTrajectory;

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
  Subscriber<Objective>& getObjectiveSubscriber();
  Publisher<Request>& getRendererPublisher();
  Publisher<Trajectory>& getTrajectoryPublisher();

protected:
  void run() override;

private:
  void optimize(double remaining_time);

  void drawGround();
  void drawRobotModel();
  void drawEnvironment();

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setMotion(const std::shared_ptr<Motion>& motion);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

  Subscriber<RobotState> robot_state_subscriber_;
  Subscriber<Objective> objective_subscriber_;
  Publisher<Request> renderer_publisher_;
  Publisher<Trajectory> trajectory_publisher_;

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;
  double trajectory_duration_;
  int trajectory_num_curves_;
  double timestep_;

  // Robot bounding volumes
  // TODO: need better data container for bounding volumes more efficient for forward kinematics and collision check
  std::vector<std::vector<std::shared_ptr<Shape>>> bounding_volumes_;

  // Objectives, updated every timestep
  std::vector<std::shared_ptr<Objective>> objectives_;

  // Optimization variables
  std::vector<double> objective_completion_times_;
  std::unique_ptr<CubicSplineTrajectory> trajectory_;

  // robot_state is nullable. For nullptr, the initial position/velocity is derived from the current spline trajectory.
  // For non-null pointer, the initial position/velocity comes from the given robot_state
  void stepForwardTrajectory(double time, std::unique_ptr<RobotState> robot_state);
};
}

#endif //DMP_PLANNER_H
