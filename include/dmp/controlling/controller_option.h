#ifndef DMP_CONTROLLER_OPTION_H
#define DMP_CONTROLLER_OPTION_H

#include <memory>

namespace dmp
{
class RobotModel;
class Motion;

class ControllerOption
{
public:
  ControllerOption();

  void setRobotModel(const std::shared_ptr<RobotModel>& robot_model) noexcept;
  const std::shared_ptr<RobotModel>& getRobotModel() const noexcept;

  void setMotion(const std::shared_ptr<Motion>& motion) noexcept;
  const std::shared_ptr<Motion>& getMotion() const noexcept;

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Motion> motion_;
};
}

#endif //DMP_CONTROLLER_OPTION_H
