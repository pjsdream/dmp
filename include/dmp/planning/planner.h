#ifndef DMP_PLANNER_H
#define DMP_PLANNER_H

#include <memory>

namespace dmp
{
class Renderer;
class RobotModel;
class Environment;
class Motion;
class Planner
{
public:
  Planner();
  ~Planner();

  void setRenderer(const std::shared_ptr<Renderer>& renderer);
  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model);
  void setMotion(const std::shared_ptr<Motion>& motion);
  void setEnvironment(const std::shared_ptr<Environment>& environment);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}

#endif //DMP_PLANNER_H
