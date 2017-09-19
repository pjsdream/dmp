#include <dmp/robot/robot_joint.h>
#include <dmp/robot/robot_link.h>

namespace dmp
{
RobotJoint::RobotJoint(JointType type) noexcept
    : type_(type)
{
}

RobotJoint::RobotJoint(const std::string& type) noexcept
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

std::string RobotJoint::getJointTypeAsString() const noexcept
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

void RobotJoint::setParentLink(const std::shared_ptr<RobotLink>& parent) noexcept
{
  parent_ = parent;
}

void RobotJoint::setChildLink(const std::shared_ptr<RobotLink>& child) noexcept
{
  child_ = child;
}

void RobotJoint::setOrigin(const Eigen::Affine3d& origin) noexcept
{
  origin_ = origin;
}

const Eigen::Affine3d& RobotJoint::getOrigin() const noexcept
{
  return origin_;
}

void RobotJoint::setAxis(const Eigen::Vector3d& axis) noexcept
{
  axis_ = axis;
}

const Eigen::Vector3d& RobotJoint::getAxis() const noexcept
{
  return axis_;
}

void RobotJoint::setName(const std::string& name) noexcept
{
  name_ = name;
}

const std::string& RobotJoint::getName() const noexcept
{
  return name_;
}

void RobotJoint::setLimit(double lower, double upper) noexcept
{
  limit_.lower = lower;
  limit_.upper = upper;
}

double RobotJoint::getLower() const noexcept
{
  return limit_.lower;
}

double RobotJoint::getUpper() const noexcept
{
  return limit_.upper;
}

Eigen::Affine3d RobotJoint::getTransform(double joint_value) const
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

std::shared_ptr<RobotLink> RobotJoint::getChildLink() const noexcept
{
  return child_;
}
}
