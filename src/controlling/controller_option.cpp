#include <dmp/controlling/controller_option.h>

namespace dmp
{
ControllerOption::ControllerOption() = default;

void ControllerOption::setRobotModel(const std::shared_ptr<RobotModel>& robot_model) noexcept
{
  robot_model_ = robot_model;
}

const std::shared_ptr<RobotModel>& ControllerOption::getRobotModel() const noexcept
{
  return robot_model_;
}

void ControllerOption::setMotion(const std::shared_ptr<Motion>& motion) noexcept
{
  motion_ = motion;
}

const std::shared_ptr<Motion>& ControllerOption::getMotion() const noexcept
{
  return motion_;
}
}
