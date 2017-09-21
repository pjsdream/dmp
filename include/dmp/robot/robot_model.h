#ifndef DMP_ROBOT_MODEL_H
#define DMP_ROBOT_MODEL_H

#include <memory>
#include <Eigen/Dense>
#include <dmp/utils/vector_eigen.h>
#include <Eigen/StdVector>

namespace dmp
{
class TreeRobotModel;
class TreeRobotLink;
class RobotLink;
class RobotJoint;

class RobotModel
{
public:
  RobotModel() = delete;
  explicit RobotModel(const TreeRobotModel& tree_robot_model) noexcept;
  ~RobotModel();

  RobotModel(const RobotModel& rhs) = default;
  RobotModel& operator=(const RobotModel& rhs) = default;

  RobotModel(RobotModel&& rhs) = default;
  RobotModel& operator=(RobotModel&& rhs) = default;

  int numLinks() const noexcept;
  std::vector<std::string> getJointNames() const;

  const RobotLink& getLink(int index) const noexcept;
  const RobotJoint& getJoint(int index) const noexcept; // index corresponds to index of link
  int getParent(int index) const noexcept;

  VectorEigen<Eigen::Affine3d> forwardKinematics(const std::vector<double>& joint_values) const;

private:
  void buildFromRobotModel(const std::shared_ptr<TreeRobotLink>& model_link, int parent_link_id = -1);

  std::vector<int> parents_;
  std::vector<RobotLink> links_;
  std::vector<RobotJoint> joints_;
};
}

#endif //DMP_ROBOT_MODEL_H
