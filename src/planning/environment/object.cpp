#include <dmp/planning/environment/object.h>
#include <dmp/shape/shape.h>

namespace dmp
{
void Object::setShape(const std::shared_ptr<Shape>& shape)
{
  shape_ = shape;
}

const std::shared_ptr<Shape>& Object::getShape() const
{
  return shape_;
}

void Object::setColor(const Eigen::Vector4f& color)
{
  color_ = color;
}

const Eigen::Vector4f& Object::getColor() const
{
  return color_;
}

void Object::setName(const std::string& name) noexcept
{
  name_ = name;
}

const std::string& Object::getName() const noexcept
{
  return name_;
}
}
