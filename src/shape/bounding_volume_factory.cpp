#include <dmp/shape/bounding_volume_factory.h>
#include <dmp/shape/shape.h>
#include <dmp/shape/aabb.h>

namespace dmp
{
std::shared_ptr<Shape> BoundingVolumeFactory::newBoundingVolumeFromPoints(const VectorEigen<Eigen::Vector3d>& points)
{
  Eigen::Vector3d min{points[0]};
  Eigen::Vector3d max{points[0]};

  for (const auto& point : points)
  {
    if (min(0) > point(0))
      min(0) = point(0);
    if (min(1) > point(1))
      min(1) = point(1);
    if (min(2) > point(2))
      min(2) = point(2);

    if (max(0) < point(0))
      max(0) = point(0);
    if (max(1) < point(1))
      max(1) = point(1);
    if (max(2) < point(2))
      max(2) = point(2);
  }

  return std::make_shared<AABB>(min, max);
}
}
