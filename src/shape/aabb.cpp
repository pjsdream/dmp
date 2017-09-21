#include <dmp/shape/aabb.h>

namespace dmp
{
AABB::AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max) noexcept
: Shape(Shape::Type::AABB), min_(min), max_(max)
{
}

const Eigen::Vector3d& AABB::getMin() const noexcept
{
  return min_;
}

const Eigen::Vector3d& AABB::getMax() const noexcept
{
  return max_;
}
}
