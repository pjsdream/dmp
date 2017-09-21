#include <dmp/robot/robot_joint.h>

namespace dmp
{
RobotJoint::RobotJoint() noexcept
    : type_(RobotJoint::Type::Undefined)
{
}

RobotJoint::RobotJoint(std::string type) noexcept
    : type_(getJointType(std::move(type)))
{
}

Eigen::Affine3d RobotJoint::getJointTransform(double joint_value) const
{
  switch (type_)
  {
    case RobotJoint::Type::Fixed:
      return origin_;
    case RobotJoint::Type::Continuous:
    case RobotJoint::Type::Revolute:
    {
      auto transform = origin_;
      return transform.rotate(Eigen::AngleAxisd(joint_value, axis_));
    }
    case RobotJoint::Type::Prismatic:
    {
      auto transform = origin_;
      return transform.translate(joint_value * axis_);
    }
    default:
      return origin_;
  }
}

RobotJoint::Type RobotJoint::getJointType(std::string type) noexcept
{
  if (type == "fixed")
    return RobotJoint::Type::Fixed;
  if (type == "continuous")
    return RobotJoint::Type::Continuous;
  if (type == "revolute")
    return RobotJoint::Type::Revolute;
  if (type == "prismatic")
    return RobotJoint::Type::Prismatic;
  return RobotJoint::Type::Undefined;
}

void RobotJoint::setName(const std::string& name) noexcept
{
  name_ = name;
}

const std::string& RobotJoint::getName() const noexcept
{
  return name_;
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

void RobotJoint::setLimits(double lower, double upper) noexcept
{
  lower_ = lower;
  upper_ = upper;
}

double RobotJoint::getLower() const noexcept
{
  return lower_;
}

double RobotJoint::getUpper() const noexcept
{
  return upper_;
}
}
