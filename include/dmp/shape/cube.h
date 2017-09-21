#ifndef DMP_CUBE_H
#define DMP_CUBE_H

#include <dmp/shape/shape.h>

namespace dmp
{
class AABB;

class Cube : public Shape
{
public:
  Cube() noexcept;
  Cube(const AABB& aabb) noexcept;
  ~Cube() override;

  void setSize(const Eigen::Vector3d& size) noexcept;
  const Eigen::Vector3d& getSize() const noexcept;

  void setTransform(const Eigen::Affine3d& transform) noexcept;
  const Eigen::Affine3d& getTransform() const noexcept;

private:
  Eigen::Affine3d transform_;
  Eigen::Vector3d size_;
};
}

#endif //DMP_CUBE_H
