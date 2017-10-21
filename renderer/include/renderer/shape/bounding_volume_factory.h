#ifndef DMP_BOUNDING_VOLUME_FACTORY_H
#define DMP_BOUNDING_VOLUME_FACTORY_H

#include <memory>
#include <Eigen/Dense>
#include <dmp/utils/vector_eigen.h>

namespace dmp
{
class Shape;

class BoundingVolumeFactory
{
public:
  static std::shared_ptr<Shape> newBoundingVolumeFromPoints(const VectorEigen<Eigen::Vector3d>& points);

private:
};
}

#endif //DMP_BOUNDING_VOLUME_FACTORY_H
