#include <dmp/robot/planning_robot_joint.h>

namespace dmp
{
PlanningRobotJoint::PlanningRobotJoint(Type type)
    : type_(type)
{
}

PlanningRobotJoint::PlanningRobotJoint(const std::string& type)
{
  if (type == "fixed")
    type_ = PlanningRobotJoint::Type::Fixed;
  else if (type == "continuous")
    type_ = PlanningRobotJoint::Type::Continuous;
  else if (type == "revolute")
    type_ = PlanningRobotJoint::Type::Revolute;
  else if (type == "prismatic")
    type_ = PlanningRobotJoint::Type::Prismatic;
  else
    type_ = PlanningRobotJoint::Type::Fixed;
}

Eigen::Affine3d PlanningRobotJoint::getJointTransform(double joint_value)
{
  switch (type_)
  {
    case PlanningRobotJoint::Type::Fixed:
      return origin_;
    case PlanningRobotJoint::Type::Continuous:
    case PlanningRobotJoint::Type::Revolute:
    {
      auto transform = origin_;
      return transform.rotate(Eigen::AngleAxisd(joint_value, axis_));
    }
    case PlanningRobotJoint::Type::Prismatic:
    {
      auto transform = origin_;
      return transform.translate(joint_value * axis_);
    }
    default:
      return origin_;
  }
}

void PlanningRobotJoint::setOrigin(const Eigen::Affine3d& origin) noexcept
{
  origin_ = origin;
}

const Eigen::Affine3d& PlanningRobotJoint::getOrigin() const noexcept
{
  return origin_;
}

void PlanningRobotJoint::setAxis(const Eigen::Vector3d& axis) noexcept
{
  axis_ = axis;
}

const Eigen::Vector3d& PlanningRobotJoint::getAxis() const noexcept
{
  return axis_;
}
}
