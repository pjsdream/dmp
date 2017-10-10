#ifndef DMP_ROBOT_JOINT_H
#define DMP_ROBOT_JOINT_H

#include <Eigen/Dense>

namespace dmp
{
class RobotJoint
{
public:
  enum class Type
  {
    Fixed,
    Continuous,
    Revolute,
    Prismatic,
    Undefined,
  };

  static Type getJointType(std::string type) noexcept;

  RobotJoint() noexcept;
  explicit RobotJoint(std::string type) noexcept;

  Eigen::Affine3d getJointTransform(double joint_value = 0.) const;

  std::string getJointTypeAsString() const noexcept;

  void setName(const std::string& name) noexcept;
  const std::string& getName() const noexcept;

  void setOrigin(const Eigen::Affine3d& origin) noexcept;
  const Eigen::Affine3d& getOrigin() const noexcept;

  void setAxis(const Eigen::Vector3d& axis) noexcept;
  const Eigen::Vector3d& getAxis() const noexcept;

  void setLimits(double lower, double upper) noexcept;
  double getLower() const noexcept;
  double getUpper() const noexcept;

private:
  Type type_;
  std::string name_;
  Eigen::Affine3d origin_;
  Eigen::Vector3d axis_;
  double lower_;
  double upper_;
};
}

#endif //DMP_ROBOT_JOINT_H
