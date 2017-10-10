#ifndef DMP_PLANNING_ROBOT_JOINT_H
#define DMP_PLANNING_ROBOT_JOINT_H

#include <Eigen/Dense>

namespace dmp
{
class PlanningRobotJoint
{
public:
  enum class Type
  {
    Fixed,
    Continuous,
    Revolute,
    Prismatic
  };

  PlanningRobotJoint() = delete;
  explicit PlanningRobotJoint(Type type);
  explicit PlanningRobotJoint(const std::string& type);

  void setOrigin(const Eigen::Affine3d& origin) noexcept;
  const Eigen::Affine3d& getOrigin() const noexcept;

  void setAxis(const Eigen::Vector3d& axis) noexcept;
  const Eigen::Vector3d& getAxis() const noexcept;

  Eigen::Affine3d getJointTransform(double joint_value);

private:
  Type type_;

  Eigen::Affine3d origin_;
  Eigen::Vector3d axis_;
};
}

#endif //DMP_PLANNING_ROBOT_JOINT_H
