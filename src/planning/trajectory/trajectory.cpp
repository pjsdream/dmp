#include <dmp/planning/trajectory/trajectory.h>
#include <dmp/planning/trajectory/trajectory_point.h>

namespace dmp
{
Trajectory::Trajectory(const std::vector<std::string>& joint_names)
: joint_names_(joint_names)
{
}

void Trajectory::addPoint(const TrajectoryPoint& point)
{
  points_.push_back(point);
}
}
