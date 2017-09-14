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
  explicit RobotModel(const std::shared_ptr<RobotLink>& root);
  ~RobotModel() = default;

  RobotModel(const RobotModel& rhs) = default;
  RobotModel& operator=(const RobotModel& rhs) = default;

  RobotModel(RobotModel&& rhs) = default;
  RobotModel& operator=(RobotModel&& rhs) = default;

  void setRoot(const std::shared_ptr<RobotLink>& root);
  const std::shared_ptr<RobotLink>& getRoot() const;

private:
  std::shared_ptr<RobotLink> root_;
};
}

#endif //DMP_ROBOT_MODEL_H
