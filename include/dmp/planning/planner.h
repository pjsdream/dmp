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
#include <dmp/planning/cost/cost.h>

namespace dmp
{
class PlanningOption;
class RobotModel;
class Motion;
class Environment;
class Shape;
class CubicSplineTrajectory;
class RobotConfigurations;

class Planner : public Node
{
public:
  Planner() = delete;
  explicit Planner(const PlanningOption& option);
  ~Planner() override;

  Planner(const Planner& rhs) = delete;
  Planner& operator=(const Planner& rhs) = delete;

  Planner(Planner&& rhs) = delete;
  Planner& operator=(Planner&& rhs) = delete;

  Subscriber<RobotState>& getRobotStateSubscriber();
  Subscriber<Objective>& getObjectiveSubscriber();
  Subscriber<Cost>& getCostSubscriber();
  Publisher<Request>& getRendererPublisher();
  Publisher<Trajectory>& getTrajectoryPublisher();

protected:
  void run() override;

private:
  void optimize(double remaining_time);

  void drawGround();
  void drawRobotModel();
  void drawEnvironment();
  void drawTrajectory();

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setMotion(const std::shared_ptr<Motion>& motion);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

  Subscriber<RobotState> robot_state_subscriber_;
  Subscriber<Objective> objective_subscriber_;
  Subscriber<Cost> cost_subscriber_;
  Publisher<Request> renderer_publisher_;
  Publisher<Trajectory> trajectory_publisher_;

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;
  double trajectory_duration_;
  int trajectory_num_curves_;
  int discretizations_;
  double timestep_;

  // Cost computation
  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeCost(const RobotConfiguration& configuration);
  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeObjectiveCost(const RobotConfiguration& configuration);

  // Robot bounding volumes
  // TODO: need better data container for bounding volumes more efficient for forward kinematics and collision check
  std::vector<std::vector<std::shared_ptr<Shape>>> bounding_volumes_;

  // Robot configurations, storing precomputed resources (e.g., forward kinematics)
  std::vector<RobotConfiguration> configurations_;

  // Objectives, updated every timestep upon request
  std::vector<std::shared_ptr<Objective>> objectives_;

  // Costs, updated every timestep upon request
  std::vector<std::shared_ptr<Cost>> costs_;

  // Optimization variables
  std::vector<double> objective_completion_times_;
  std::unique_ptr<CubicSplineTrajectory> trajectory_;

  // robot_state is nullable. For nullptr, the initial position/velocity is derived from the current spline trajectory.
  // For non-null pointer, the initial position/velocity comes from the given robot_state
  void stepForwardTrajectory(double time, std::unique_ptr<RobotState> robot_state);
};
}

#endif //DMP_PLANNER_H
