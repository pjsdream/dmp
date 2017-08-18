#include <dmp/planning/environment/object.h>
#include <dmp/shape/shape.h>

namespace dmp
{
void Object::setShape(const std::shared_ptr<Shape>& shape)
{
  shape_ = shape;
}

void Object::setColor(const Eigen::Vector4f& color)
{
  color_ = color;
}

const std::shared_ptr<Shape>& Object::getShape()
{
  return shape_;
}

const Eigen::Vector4f& Object::getColor()
{
  return color_;
}
}