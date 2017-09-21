#include <dmp/robot/tree_robot_joint.h>
#include <dmp/robot/tree_robot_link.h>

namespace dmp
{
TreeRobotJoint::TreeRobotJoint(JointType type) noexcept
    : type_(type)
{
}

TreeRobotJoint::TreeRobotJoint(const std::string& type) noexcept
{
  if (type == "fixed")
    type_ = JointType::Fixed;
  else if (type == "continuous")
    type_ = JointType::Continuous;
  else if (type == "revolute")
    type_ = JointType::Revolute;
  else if (type == "prismatic")
    type_ = JointType::Prismatic;
}

std::string TreeRobotJoint::getJointTypeAsString() const noexcept
{
  switch (type_)
  {
    case JointType::Fixed:
      return "fixed";
    case JointType::Continuous:
      return "continuous";
    case JointType::Revolute:
      return "revolute";
    case JointType::Prismatic:
      return "prismatic";
  }
}

void TreeRobotJoint::setParentLink(const std::shared_ptr<TreeRobotLink>& parent) noexcept
{
  parent_ = parent;
}

void TreeRobotJoint::setChildLink(const std::shared_ptr<TreeRobotLink>& child) noexcept
{
  child_ = child;
}

void TreeRobotJoint::setOrigin(const Eigen::Affine3d& origin) noexcept
{
  origin_ = origin;
}

const Eigen::Affine3d& TreeRobotJoint::getOrigin() const noexcept
{
  return origin_;
}

void TreeRobotJoint::setAxis(const Eigen::Vector3d& axis) noexcept
{
  axis_ = axis;
}

const Eigen::Vector3d& TreeRobotJoint::getAxis() const noexcept
{
  return axis_;
}

void TreeRobotJoint::setName(const std::string& name) noexcept
{
  name_ = name;
}

const std::string& TreeRobotJoint::getName() const noexcept
{
  return name_;
}

void TreeRobotJoint::setLimit(double lower, double upper) noexcept
{
  limit_.lower = lower;
  limit_.upper = upper;
}

double TreeRobotJoint::getLower() const noexcept
{
  return limit_.lower;
}

double TreeRobotJoint::getUpper() const noexcept
{
  return limit_.upper;
}

Eigen::Affine3d TreeRobotJoint::getTransform(double joint_value) const
{
  Eigen::Affine3d transform = origin_;
  switch (type_)
  {
    case JointType::Fixed:
      break;
    case JointType::Continuous:
    case JointType::Revolute:
      transform.rotate(Eigen::AngleAxisd(joint_value, axis_));
      break;
    case JointType::Prismatic:
      transform.translate(joint_value * axis_);
      break;
  }
  return transform;
}

std::shared_ptr<TreeRobotLink> TreeRobotJoint::getChildLink() const noexcept
{
  return child_;
}
}
