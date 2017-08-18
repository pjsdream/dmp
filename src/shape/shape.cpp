#include <dmp/shape/shape.h>

namespace dmp
{
Shape::Shape()
    : transform_(Eigen::Affine3d::Identity())
{
}

Shape::~Shape() = default;

void Shape::setTransform(const Eigen::Affine3d& transform)
{
  transform_ = transform;
}

const Eigen::Affine3d& Shape::getTransform()
{
  return transform_;
}
}
