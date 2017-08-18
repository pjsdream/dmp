#ifndef DMP_SPHERE_H
#define DMP_SPHERE_H

#include <dmp/shape/shape.h>

namespace dmp
{
class Sphere : public Shape
{
public:
  Sphere();
  ~Sphere() override;

  void setRadius(double radius);

  double getRadius();

private:
  double radius_;
};
}

#endif //DMP_SPHERE_H
