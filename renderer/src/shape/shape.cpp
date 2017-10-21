#include <dmp/shape/shape.h>
#include <dmp/shape/distance_query.h>

namespace dmp
{
Shape::Shape(Type type) noexcept
    : type_(type)
{
}

Shape::~Shape() = default;

Shape::Type Shape::getType() const noexcept
{
  return type_;
}
}
