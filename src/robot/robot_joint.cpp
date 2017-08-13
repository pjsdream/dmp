#include <dmp/robot/robot_joint.h>
#include <dmp/robot/robot_link.h>

namespace dmp
{
RobotJoint::RobotJoint(JointType type)
    : type_(type)
{
}

RobotJoint::RobotJoint(const std::string& type)
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

void RobotJoint::setParentLink(const std::shared_ptr<RobotLink>& parent)
{
  parent_ = parent;
}

void RobotJoint::setChildLink(const std::shared_ptr<RobotLink>& child)
{
  child_ = child;
}

void RobotJoint::setOrigin(const Eigen::Affine3d& origin)
{
  origin_ = origin;
}

void RobotJoint::setAxis(const Eigen::Vector3d& axis)
{
  axis_ = axis;
}

void RobotJoint::setName(const std::string& name)
{
  name_ = name;
}

void RobotJoint::setLimit(double lower, double upper)
{
  limit_.lower = lower;
  limit_.upper = upper;
}

Eigen::Affine3d RobotJoint::getTransform(double joint_value)
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
      transform.translate(axis_ * joint_value);
      break;
  }
  return transform;
}

std::shared_ptr<RobotLink> RobotJoint::getChildLink()
{
  return child_;
}
}
