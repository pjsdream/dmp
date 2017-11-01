#include <planner/shape/sphere.h>

namespace dmp
{
Sphere::Sphere() noexcept
    : Shape(Shape::Type::Sphere), radius_(1.), position_(Eigen::Vector3d::Zero())
{
}

Sphere::~Sphere() = default;

void Sphere::setPosition(const Eigen::Vector3d& position) noexcept
{
  position_ = position;
}

const Eigen::Vector3d& Sphere::getPosition() const noexcept
{
  return position_;
}

void Sphere::setRadius(double radius) noexcept
{
  radius_ = radius;
}

double Sphere::getRadius() const noexcept
{
  return radius_;
}
}
