#ifndef DMP_PLANNING_OPTION_H
#define DMP_PLANNING_OPTION_H

#include <memory>

namespace dmp
{
class RobotModel;
class Environment;
class Motion;

class PlanningOption
{
public:
  PlanningOption();
  ~PlanningOption();

  PlanningOption(const PlanningOption& rhs) = default;
  PlanningOption& operator=(const PlanningOption& rhs) = default;

  PlanningOption(PlanningOption&& rhs) = default;
  PlanningOption& operator=(PlanningOption&& rhs) = default;

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  std::shared_ptr<RobotModel> getRobotModel() const;

  void setEnvironment(const std::shared_ptr<Environment>& environment);
  std::shared_ptr<Environment> getEnvironment() const;

  void setMotion(const std::shared_ptr<Motion>& motion);
  std::shared_ptr<Motion> getMotion() const;

  void setTrajectoryOptions(double duration, int num_curves) noexcept;
  double getTrajectoryDuration() const noexcept;
  int numTrajectoryCurves() const noexcept;

  void setTimestep(double timestep) noexcept;
  double getTimestep() const noexcept;

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;
  double trajectory_duration_;
  int trajectory_num_curves_;
  double timestep_;
};
}

#endif //DMP_PLANNING_OPTION_H
