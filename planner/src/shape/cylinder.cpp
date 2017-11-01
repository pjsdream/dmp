#include <planner/shape/cylinder.h>

namespace dmp
{
Cylinder::Cylinder() noexcept
    : Shape(Shape::Type::Cylinder), radius_(1.), height_(1.)
{
}

Cylinder::~Cylinder() = default;

void Cylinder::setTransform(const Eigen::Affine3d& transform) noexcept
{
  transform_ = transform;
}

const Eigen::Affine3d& Cylinder::getTransform() const noexcept
{
  return transform_;
}

void Cylinder::setDimension(double radius, double height) noexcept
{
  radius_ = radius;
  height_ = height;
}

double Cylinder::getRadius() const noexcept
{
  return radius_;
}

double Cylinder::getHeight() const noexcept
{
  return height_;
}
}
