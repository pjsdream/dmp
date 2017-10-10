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

  void setPositions(const Eigen::VectorXd& positions) noexcept;
  const Eigen::VectorXd& getPositions() const noexcept;

  void setVelocities(const Eigen::VectorXd& velocities) noexcept;
  const Eigen::VectorXd& getVelocities() const noexcept;

  void forwardKinematics();

  const Eigen::Affine3d& getTransform(int link_index) const;

private:
  std::shared_ptr<PlanningRobotModel> robot_model_;

  Eigen::VectorXd positions_;
  Eigen::VectorXd velocities_;

  VectorEigen<Eigen::Affine3d> transforms_;
};
}

#endif //DMP_ROBOT_CONFIGURATION_H
