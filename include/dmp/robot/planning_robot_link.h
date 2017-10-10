#ifndef DMP_PLANNING_ROBOT_LINK_H
#define DMP_PLANNING_ROBOT_LINK_H

#include <vector>
#include <memory>

namespace dmp
{
class Shape;

class PlanningRobotLink
{
public:
  void addBoundingVolume(const std::shared_ptr<Shape>& shape);
  const std::vector<std::shared_ptr<Shape>>& getBoundingVolumes() const noexcept;

private:
  std::vector<std::shared_ptr<Shape>> bounding_volumes_;
};
}

#endif //DMP_PLANNING_ROBOT_LINK_H
