#include <planner/shape/shape.h>
#include <planner/shape/distance_query.h>

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
