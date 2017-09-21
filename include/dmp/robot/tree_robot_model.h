#ifndef DMP_TREE_ROBOT_MODEL_H
#define DMP_TREE_ROBOT_MODEL_H

#include <memory>

namespace dmp
{
class TreeRobotLink;
class TreeRobotModel
{
public:
  TreeRobotModel() = default;
  explicit TreeRobotModel(const std::shared_ptr<TreeRobotLink>& root);
  ~TreeRobotModel() = default;

  TreeRobotModel(const TreeRobotModel& rhs) = default;
  TreeRobotModel& operator=(const TreeRobotModel& rhs) = default;

  TreeRobotModel(TreeRobotModel&& rhs) = default;
  TreeRobotModel& operator=(TreeRobotModel&& rhs) = default;

  void setRoot(const std::shared_ptr<TreeRobotLink>& root);
  const std::shared_ptr<TreeRobotLink>& getRoot() const;

private:
  std::shared_ptr<TreeRobotLink> root_;
};
}

#endif //DMP_TREE_ROBOT_MODEL_H
