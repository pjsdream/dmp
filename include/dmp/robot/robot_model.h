#ifndef DMP_ROBOT_MODEL_H
#define DMP_ROBOT_MODEL_H

#include <memory>

namespace dmp
{
class RobotLink;
class RobotModel
{
public:
  RobotModel() = default;
  RobotModel(const std::shared_ptr<RobotLink>& root);
  ~RobotModel() = default;

  void setRoot(const std::shared_ptr<RobotLink>& root);
  std::shared_ptr<RobotLink> getRoot();

private:
  std::shared_ptr<RobotLink> root_;
};
}

#endif //DMP_ROBOT_MODEL_H
