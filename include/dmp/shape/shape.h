#ifndef DMP_SHAPE_H
#define DMP_SHAPE_H

#include <Eigen/Dense>
#include <memory>

namespace dmp
{
class DistanceQuery;

class Shape
{
public:
  enum class Type
  {
    AABB,
    Cube,
    Cylinder,
    Sphere,
  };

  Shape() = delete;
  explicit Shape(Type type) noexcept;
  virtual ~Shape();

  Type getType() const noexcept;

  // Precondition: downcast is possible.
  template<typename T>
  const T& as() const
  {
    return static_cast<const T&>(*this);
  }

private:
  const Type type_;
};
}

#endif //DMP_SHAPE_H
