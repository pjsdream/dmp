#ifndef DMP_PLANNING_ROBOT_MODEL_H
#define DMP_PLANNING_ROBOT_MODEL_H

#include <vector>
#include <string>
#include <memory>
#include <unordered_set>

#include <Eigen/Dense>

namespace dmp
{
class RobotModel;
class PlanningRobotJoint;
class PlanningRobotLink;
class Motion;

//
// Robot model with n joints.
// Link index: 0~n
// Joint index: 0~(n-1)
// Joint i correspond to link index (i+1).
// Parent(i): parent link index of joint i.
//
class PlanningRobotModel
{
public:
  PlanningRobotModel() = delete;
  PlanningRobotModel(const std::shared_ptr<RobotModel>& robot_model, const std::shared_ptr<Motion>& motion);

  int numLinks() const noexcept;
  int numJoints() const noexcept;
  const PlanningRobotLink& getLink(int index) const noexcept;
  const PlanningRobotJoint& getJoint(int index) const noexcept;
  int getParentLinkIndex(int joint_index) const noexcept;

  int getGripperLinkIndex() const noexcept;
  const Eigen::Affine3d& getGripperTransform() const noexcept;
  const std::vector<int>& getLinkIndicesToGripper() const noexcept;

private:
  std::vector<int> parents_;
  std::vector<PlanningRobotLink> links_;
  std::vector<PlanningRobotJoint> joints_;

  int gripper_link_index_;
  Eigen::Affine3d gripper_transform_;

  // Index chain from base to gripper
  std::vector<int> link_indices_to_gripper_;
};
}

#endif //DMP_PLANNING_ROBOT_MODEL_H
