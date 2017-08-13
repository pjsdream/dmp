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
  explicit RobotJoint(JointType type);
  explicit RobotJoint(const std::string& type);
  ~RobotJoint() = default;

  void setParentLink(const std::shared_ptr<RobotLink>& parent);
  void setChildLink(const std::shared_ptr<RobotLink>& child);

  void setName(const std::string& name);
  void setOrigin(const Eigen::Affine3d& origin);
  void setAxis(const Eigen::Vector3d& axis);
  void setLimit(double lower, double upper);

  Eigen::Affine3d getTransform(double joint_value);

  std::shared_ptr<RobotLink> getChildLink();

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
