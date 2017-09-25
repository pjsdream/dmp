#ifndef DMP_TRAJECTORY_H
#define DMP_TRAJECTORY_H

#include <vector>
#include <string>

#include <dmp/comm/message.h>
#include <dmp/trajectory/trajectory_point.h>

namespace dmp
{
class Trajectory : public Message
{
public:
  Trajectory() = delete;
  explicit Trajectory(const std::vector<std::string>& joint_names);
  ~Trajectory() override = default;

  Trajectory(const Trajectory& rhs) = default;
  Trajectory& operator=(const Trajectory& rhs) = default;

  Trajectory(Trajectory&& rhs) = default;
  Trajectory& operator=(Trajectory&& rhs) = default;

  const std::vector<std::string>& getJointNames() const noexcept;

  void addPoint(const TrajectoryPoint& point) noexcept;

  const std::vector<TrajectoryPoint>& getPoints() const noexcept;

private:
  std::vector<std::string> joint_names_;
  std::vector<TrajectoryPoint> points_;
};
}

#endif //DMP_TRAJECTORY_H
