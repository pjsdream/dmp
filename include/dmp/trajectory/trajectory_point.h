#ifndef DMP_TRAJECTORY_POINT_H
#define DMP_TRAJECTORY_POINT_H

#include <Eigen/Dense>

namespace dmp
{
class TrajectoryPoint
{
public:
  TrajectoryPoint() = delete;
  TrajectoryPoint(double time, const Eigen::VectorXd& joint_positions_) noexcept;
  TrajectoryPoint(double time, const Eigen::Affine3d& base, const Eigen::VectorXd& joint_positions_) noexcept;

  double getTime() const noexcept;
  const Eigen::Affine3d& getBase() const noexcept;
  const Eigen::VectorXd& getJointPositions() const noexcept;

private:
  double time_;
  Eigen::Affine3d base_;
  Eigen::VectorXd joint_positions_;
};
}

#endif //DMP_TRAJECTORY_POINT_H
