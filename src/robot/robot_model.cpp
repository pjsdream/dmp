#include <dmp/robot/robot_model.h>
#include <dmp/robot/robot_link.h>

namespace dmp
{
RobotModel::RobotModel(const std::shared_ptr<RobotLink>& root)
    : root_(root)
{
}

void RobotModel::setRoot(const std::shared_ptr<RobotLink>& root)
{
  root_ = root;
}

const std::shared_ptr<RobotLink>& RobotModel::getRoot() const
{
  return root_;
}
}
