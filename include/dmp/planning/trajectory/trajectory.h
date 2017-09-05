#ifndef DMP_TRAJECTORY_H
#define DMP_TRAJECTORY_H

#include <vector>
#include <string>

namespace dmp
{
class TrajectoryPoint;
class Trajectory
{
public:
  Trajectory() = delete;
  Trajectory(const std::vector<std::string>& joint_names);

  void addPoint(const TrajectoryPoint& point);

private:
  std::vector<std::string> joint_names_;
  std::vector<TrajectoryPoint> points_;
};
}

#endif //DMP_TRAJECTORY_H
