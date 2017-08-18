#include <dmp/shape/cylinder.h>

namespace dmp
{
Cylinder::Cylinder()
    : Shape(), radius_(1.), height_(1.)
{
}

Cylinder::~Cylinder() = default;

void Cylinder::setDimension(double radius, double height)
{
  radius_ = radius;
  height_ = height;
}

double Cylinder::getRadius()
{
  return radius_;
}

double Cylinder::getHeight()
{
  return height_;
}
}
