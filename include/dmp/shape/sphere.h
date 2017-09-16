#ifndef DMP_SPHERE_H
#define DMP_SPHERE_H

#include <dmp/shape/shape.h>

namespace dmp
{
class Sphere : public Shape
{
public:
  Sphere() noexcept;
  ~Sphere() override;

  void setPosition(const Eigen::Vector3d& position) noexcept;
  const Eigen::Vector3d& getPosition() const noexcept;

  void setRadius(double radius) noexcept;
  double getRadius() const noexcept;

private:
  Eigen::Vector3d position_;
  double radius_;
};
}

#endif //DMP_SPHERE_H
