#ifndef DMP_ROBOT_CONFIGURATION_H
#define DMP_ROBOT_CONFIGURATION_H

#include <dmp/utils/vector_eigen.h>

#include <memory>
#include <unordered_map>

#include <Eigen/Dense>

namespace dmp
{
class RobotModel;
class Motion;

class RobotConfiguration
{
public:
  RobotConfiguration() = delete;

  // Active joints are the motion body joint names
  RobotConfiguration(const std::shared_ptr<RobotModel>& robot_model,
                     const std::shared_ptr<Motion>& motion);

  const std::shared_ptr<Motion>& getMotion() const noexcept;

  void setPositions(const Eigen::VectorXd& positions) noexcept;
  const Eigen::VectorXd& getPositions() const noexcept;

  void setVelocities(const Eigen::VectorXd& velocities) noexcept;
  const Eigen::VectorXd& getVelocities() const noexcept;

  void forwardKinematics();

  const Eigen::Affine3d& getTransform(const std::string& link_name) const;

private:
  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<Motion> motion_;
  std::unordered_map<std::string, int> joint_names_to_index_;

  Eigen::VectorXd positions_;
  Eigen::VectorXd velocities_;

  VectorEigen<Eigen::Affine3d> transforms_;
};
}

#endif //DMP_ROBOT_CONFIGURATION_H
