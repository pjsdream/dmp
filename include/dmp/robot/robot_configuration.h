#ifndef DMP_ROBOT_CONFIGURATION_H
#define DMP_ROBOT_CONFIGURATION_H

#include <dmp/utils/vector_eigen.h>

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>

namespace dmp
{
class PlanningRobotModel;
class Motion;

class RobotConfiguration
{
public:
  RobotConfiguration() = delete;

  // All joints are active joints.
  explicit RobotConfiguration(const std::shared_ptr<PlanningRobotModel>& robot_model);

  std::shared_ptr<PlanningRobotModel> getRobotModel() const;

  void setPositions(const Eigen::VectorXd& positions) noexcept;
  const Eigen::VectorXd& getPositions() const noexcept;

  void setVelocities(const Eigen::VectorXd& velocities) noexcept;
  const Eigen::VectorXd& getVelocities() const noexcept;

  // Precondition: setPositions()
  void forwardKinematics();
  const Eigen::Affine3d& getTransform(int link_index) const;
  Eigen::Affine3d getGripperTransformFromBase() const;

  // Precondition: forwardKinematics()
  void computeGripperTransformDerivative();
  Eigen::Matrix4d getGripperDerivativeTransform(int joint_index) const;

private:
  std::shared_ptr<PlanningRobotModel> robot_model_;

  Eigen::VectorXd positions_;
  Eigen::VectorXd velocities_;

  VectorEigen<Eigen::Affine3d> transforms_; // link_index
  VectorEigen<Eigen::Matrix4d> gripper_transform_derivatives_; // joint_index
};
}

#endif //DMP_ROBOT_CONFIGURATION_H
