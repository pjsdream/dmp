#include <include/dmp/trajectory/trajectory.h>
#include <include/dmp/trajectory/trajectory_point.h>

namespace dmp
{
Trajectory::Trajectory(const std::vector<std::string>& joint_names)
: joint_names_(joint_names)
{
}

const std::vector<std::string>& Trajectory::getJointNames() const noexcept
{
  return joint_names_;
}

void Trajectory::addPoint(const TrajectoryPoint& point) noexcept
{
  points_.push_back(point);
}

const std::vector<TrajectoryPoint>& Trajectory::getPoints() const noexcept
{
  return points_;
}
}
