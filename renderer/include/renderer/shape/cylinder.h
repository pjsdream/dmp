#ifndef DMP_CYLINDER_H
#define DMP_CYLINDER_H

#include <dmp/shape/shape.h>

namespace dmp
{
class Cylinder : public Shape
{
public:
  Cylinder() noexcept;
  ~Cylinder() override;

  void setTransform(const Eigen::Affine3d& transform) noexcept;
  const Eigen::Affine3d& getTransform() const noexcept;

  void setDimension(double radius, double height) noexcept;
  double getRadius() const noexcept;
  double getHeight() const noexcept;

private:
  Eigen::Affine3d transform_;
  double radius_;
  double height_;
};
}

#endif //DMP_CYLINDER_H
