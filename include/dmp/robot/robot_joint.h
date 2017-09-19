#ifndef DMP_ROBOT_JOINT_H
#define DMP_ROBOT_JOINT_H

#include <memory>

#include <Eigen/Dense>

namespace dmp
{
class RobotLink;
class RobotJoint
{
public:
  enum class JointType
  {
    Fixed,
    Continuous,
    Revolute,
    Prismatic,
  };

  RobotJoint() = delete;
  explicit RobotJoint(JointType type) noexcept;
  explicit RobotJoint(const std::string& type) noexcept;
  ~RobotJoint() = default;

  std::string getJointTypeAsString() const noexcept;

  void setParentLink(const std::shared_ptr<RobotLink>& parent) noexcept;
  void setChildLink(const std::shared_ptr<RobotLink>& child) noexcept;

  void setName(const std::string& name) noexcept;
  const std::string& getName() const noexcept;

  void setOrigin(const Eigen::Affine3d& origin) noexcept;
  const Eigen::Affine3d& getOrigin() const noexcept;

  void setAxis(const Eigen::Vector3d& axis) noexcept;
  const Eigen::Vector3d& getAxis() const noexcept;

  void setLimit(double lower, double upper) noexcept;
  double getLower() const noexcept;
  double getUpper() const noexcept;

  Eigen::Affine3d getTransform(double joint_value) const;

  std::shared_ptr<RobotLink> getChildLink() const noexcept;

private:
  JointType type_;

  std::weak_ptr<RobotLink> parent_;
  std::shared_ptr<RobotLink> child_;

  std::string name_;
  Eigen::Affine3d origin_;
  Eigen::Vector3d axis_;

  struct Limit
  {
    double lower;
    double upper;
  };
  Limit limit_;
};
}

#endif //DMP_ROBOT_JOINT_H
