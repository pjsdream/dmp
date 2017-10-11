#include <dmp/robot/planning_robot_joint.h>

#include <iostream>

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

Eigen::Affine3d PlanningRobotJoint::getJointTransform(double joint_value) const noexcept
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
  }
}

Eigen::Matrix4d PlanningRobotJoint::getJointDerivativeTransform(double joint_value) const noexcept
{
  switch (type_)
  {
    case PlanningRobotJoint::Type::Fixed:
      return Eigen::Matrix4d::Zero();
    case PlanningRobotJoint::Type::Continuous:
    case PlanningRobotJoint::Type::Revolute:
    {
      const double s = std::sin(joint_value / 2.);
      const double c = std::cos(joint_value / 2.);

      const double w = c;
      const double x = s * axis_(0);
      const double y = s * axis_(1);
      const double z = s * axis_(2);

      const double dw = -s / 2.;
      const double dx = c * axis_(0) / 2.;
      const double dy = c * axis_(1) / 2.;
      const double dz = c * axis_(2) / 2.;

      Eigen::Matrix4d der = Eigen::Matrix4d::Zero();
      der(0, 0) = -4. * (y * dy + z * dz);
      der(1, 0) = 2. * (dx * y + x * dy + dz * w + z * dw);
      der(2, 0) = 2. * (dx * z + x * dz - dy * w - y * dw);
      der(0, 1) = 2. * (dx * y + x * dy - dz * w - z * dw);
      der(1, 1) = -4. * (x * dx + z * dz);
      der(2, 1) = 2. * (dy * z + y * dz + dx * w + x * dw);
      der(0, 2) = 2. * (dx * z + x * dz + dy * w + y * dw);
      der(1, 2) = 2. * (dy * z + y * dz - dx * w - x * dw);
      der(2, 2) = -4. * (x * dx + y * dy);

      return origin_ * der;
    }
    case PlanningRobotJoint::Type::Prismatic:
    {
      Eigen::Matrix4d der = Eigen::Matrix4d::Zero();
      der.block(0, 3, 3, 1) = axis_;
      return origin_ * der;
    }
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
