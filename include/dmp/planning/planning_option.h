#ifndef DMP_PLANNING_OPTION_H
#define DMP_PLANNING_OPTION_H

#include <memory>

namespace dmp
{
class RobotModel;
class Environment;
class Motion;
class Renderer;
class PlanningOption
{
public:
  PlanningOption();
  ~PlanningOption();

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  std::shared_ptr<RobotModel> getRobotModel() const;

  void setEnvironment(const std::shared_ptr<Environment>& environment);
  std::shared_ptr<Environment> getEnvironment() const;

  void setMotion(const std::shared_ptr<Motion>& motion);
  std::shared_ptr<Motion> getMotion() const;

  void setRenderer(const std::shared_ptr<Renderer>& renderer);
  std::shared_ptr<Renderer> getRenderer() const;

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Environment> environment_;
  std::shared_ptr<Motion> motion_;
  std::shared_ptr<Renderer> renderer_;
};
}

#endif //DMP_PLANNING_OPTION_H
