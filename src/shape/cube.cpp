#include <dmp/shape/cube.h>

namespace dmp
{
Cube::Cube()
: Shape(), size_(Eigen::Vector3d(1., 1., 1.))
{
}

Cube::~Cube() = default;

void Cube::setSize(const Eigen::Vector3d& size)
{
  size_ = size;
}

const Eigen::Vector3d& Cube::getSize()
{
  return size_;
}
}
