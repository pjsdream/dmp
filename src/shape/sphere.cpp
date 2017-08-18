#include <dmp/shape/sphere.h>

namespace dmp
{
Sphere::Sphere()
    : Shape(), radius_(1.)
{
}

Sphere::~Sphere() = default;

void Sphere::setRadius(double radius)
{
  radius_ = radius;
}

double Sphere::getRadius()
{
  return radius_;
}
}
