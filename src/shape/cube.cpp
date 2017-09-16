#include <dmp/shape/cube.h>

namespace dmp
{
Cube::Cube() noexcept
    : Shape(Shape::Type::Cube), size_(Eigen::Vector3d(1., 1., 1.)), transform_(Eigen::Affine3d::Identity())
{
}

Cube::~Cube() = default;

void Cube::setTransform(const Eigen::Affine3d& transform) noexcept
{
  transform_ = transform;
}

const Eigen::Affine3d& Cube::getTransform() const noexcept
{
  return transform_;
}

void Cube::setSize(const Eigen::Vector3d& size) noexcept
{
  size_ = size;
}

const Eigen::Vector3d& Cube::getSize() const noexcept
{
  return size_;
}
}
