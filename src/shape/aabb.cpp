#include <dmp/shape/aabb.h>

namespace dmp
{
AABB::AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max) noexcept
: Shape(Shape::Type::AABB), min_(min), max_(max)
{
}
}
