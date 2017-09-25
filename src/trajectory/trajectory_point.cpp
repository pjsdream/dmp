#include <include/dmp/trajectory/trajectory_point.h>

namespace dmp
{
TrajectoryPoint::TrajectoryPoint(double time, const Eigen::VectorXd& joint_positions) noexcept
    : time_(time), base_(Eigen::Affine3d::Identity()), joint_positions_(joint_positions)
{
}

TrajectoryPoint::TrajectoryPoint(double time,
                                 const Eigen::Affine3d& base,
                                 const Eigen::VectorXd& joint_positions) noexcept
    : time_(time), base_(base), joint_positions_(joint_positions)
{
}

double TrajectoryPoint::getTime() const noexcept
{
  return time_;
}

const Eigen::Affine3d& TrajectoryPoint::getBase() const noexcept
{
  return base_;
}

const Eigen::VectorXd& TrajectoryPoint::getJointPositions() const noexcept
{
  return joint_positions_;
}
}
