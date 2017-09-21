#include <dmp/robot/tree_robot_model.h>
#include <dmp/robot/tree_robot_link.h>

namespace dmp
{
TreeRobotModel::TreeRobotModel(const std::shared_ptr<TreeRobotLink>& root)
    : root_(root)
{
}

void TreeRobotModel::setRoot(const std::shared_ptr<TreeRobotLink>& root)
{
  root_ = root;
}

const std::shared_ptr<TreeRobotLink>& TreeRobotModel::getRoot() const
{
  return root_;
}
}
