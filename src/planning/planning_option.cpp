#include <dmp/planning/planning_option.h>

namespace dmp
{
PlanningOption::PlanningOption() = default;

PlanningOption::~PlanningOption() = default;

void PlanningOption::setRobotModel(const std::shared_ptr<RobotModel>& robot_model)
{
  robot_model_ = robot_model;
}

std::shared_ptr<RobotModel> PlanningOption::getRobotModel() const
{
  return robot_model_;
}

void PlanningOption::setEnvironment(const std::shared_ptr<Environment>& environment)
{
  environment_ = environment;
}

std::shared_ptr<Environment> PlanningOption::getEnvironment() const
{
  return environment_;
}

void PlanningOption::setMotion(const std::shared_ptr<Motion>& motion)
{
  motion_ = motion;
}

std::shared_ptr<Motion> PlanningOption::getMotion() const
{
  return motion_;
}

void PlanningOption::setRenderer(const std::shared_ptr<Renderer>& renderer)
{
  renderer_ = renderer;
}

std::shared_ptr<Renderer> PlanningOption::getRenderer() const
{
  return renderer_;
}
}