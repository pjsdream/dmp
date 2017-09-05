#include <dmp/planning/trajectory/trajectory_point.h>

namespace dmp
{
TrajectoryPoint::TrajectoryPoint(double time, const Eigen::VectorXd& joint_positions)
    : time_(time), base_(Eigen::Affine3d::Identity()), joint_positions_(joint_positions)
{
}

TrajectoryPoint::TrajectoryPoint(double time, const Eigen::Affine3d& base, const Eigen::VectorXd& joint_positions)
    : time_(time), base_(base), joint_positions_(joint_positions)
{
}
}
