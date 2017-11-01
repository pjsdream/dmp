#include <planner/robot/planning_robot_link.h>

namespace dmp
{
void PlanningRobotLink::addBoundingVolume(const std::shared_ptr<Shape>& shape)
{
  bounding_volumes_.push_back(shape);
}

const std::vector<std::shared_ptr<Shape>>& PlanningRobotLink::getBoundingVolumes() const noexcept
{
  return bounding_volumes_;
}
}
