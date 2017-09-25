#include <dmp/robot/robot_model.h>
#include <dmp/robot/tree_robot_model.h>
#include <dmp/robot/tree_robot_link.h>
#include <dmp/robot/tree_robot_joint.h>
#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>

namespace dmp
{
RobotModel::RobotModel(const TreeRobotModel& tree_robot_model) noexcept
{
  buildFromRobotModel(tree_robot_model.getRoot());
}

RobotModel::~RobotModel() = default;

void RobotModel::buildFromRobotModel(const std::shared_ptr<TreeRobotLink>& model_link, int parent_link_id)
{
  // create link
  RobotLink link(model_link->getName());

  for (const auto& model_visual : model_link->getVisuals())
    link.addVisual(model_visual.filename, model_visual.transform, model_visual.has_color, model_visual.color);

  for (const auto& model_collision : model_link->getCollisions())
    link.addCollision(model_collision.filename, model_collision.transform);

  links_.push_back(std::move(link));
  parents_.push_back(parent_link_id);
  auto link_id = static_cast<int>(links_.size() - 1);

  // for root node, add a dummy joint (undefined joint)
  if (link_id == 0)
    joints_.resize(1);

  // traverse child joints
  for (const auto& model_child_joint : model_link->getChildJoints())
  {
    // create joint
    RobotJoint joint(model_child_joint->getJointTypeAsString());
    joint.setName(model_child_joint->getName());
    joint.setOrigin(model_child_joint->getOrigin());
    joint.setAxis(model_child_joint->getAxis());
    joint.setLimits(model_child_joint->getLower(), model_child_joint->getUpper());

    joints_.push_back(std::move(joint));
    joint_name_to_index_[model_child_joint->getName()] = static_cast<int>(joints_.size() - 1);

    buildFromRobotModel(model_child_joint->getChildLink(), link_id);
  }
}

int RobotModel::numLinks() const noexcept
{
  return static_cast<int>(links_.size());
}

int RobotModel::numJoints() const noexcept
{
  return numLinks() - 1;
}

std::vector<std::string> RobotModel::getJointNames() const
{
  std::vector<std::string> joint_names;
  joint_names.reserve(numLinks() - 1);

  auto it = joints_.cbegin();
  for (it++; it != joints_.cend(); it++)
    joint_names.emplace_back(it->getName());

  return joint_names;
}

VectorEigen<Eigen::Affine3d> RobotModel::forwardKinematics(const std::vector<double>& joint_values) const
{
  VectorEigen<Eigen::Affine3d> transforms;
  transforms.reserve(numLinks());

  transforms.emplace_back(Eigen::Affine3d::Identity());

  for (int i = 1; i < numLinks(); i++)
  {
    const auto& parent = parents_[i];
    const auto& link = links_[i];
    const auto& joint = joints_[i];

    transforms.emplace_back(transforms[parent] * joint.getJointTransform(joint_values[i - 1]));
  }

  return transforms;
}

const RobotLink& RobotModel::getLink(int index) const noexcept
{
  return links_[index];
}

const RobotJoint& RobotModel::getJoint(int index) const noexcept
{
  return joints_[index];
}

const RobotJoint& RobotModel::getJoint(const std::string& joint_name) const noexcept
{
  return joints_[joint_name_to_index_.find(joint_name)->second];
}

int RobotModel::getParent(int index) const noexcept
{
  return parents_[index];
}
}
