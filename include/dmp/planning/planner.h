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
class PlanningRobotModel;

class Planner : public Node
{
public:
  Planner() = delete;
  Planner(const std::shared_ptr<Manager>& manager, const PlanningOption& option);
  ~Planner() override;

  Planner(const Planner& rhs) = delete;
  Planner& operator=(const Planner& rhs) = delete;

  Planner(Planner&& rhs) = delete;
  Planner& operator=(Planner&& rhs) = delete;

protected:
  void run() override;

private:
  void optimize(double remaining_time);

  void drawGround();
  void drawRobotModel();
  void drawEnvironment();
  void drawTrajectory();
  void drawRobotStatus(const Eigen::VectorXd& p, std::string tag);

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);

  void createPlanningRobotModel();
  std::shared_ptr<PlanningRobotModel> planning_robot_model_;

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
  std::tuple<double, Eigen::VectorXd, Eigen::VectorXd> computeObjectiveCost(const RobotConfiguration& configuration,
                                                                            int objective_index);

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
