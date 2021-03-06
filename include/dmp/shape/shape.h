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

  // Precondition: downcast is possible, as the object actually refers to the derived class
  template<typename T, typename = typename std::enable_if_t<std::is_base_of<Shape, T>::value>>
  const T& as() const
  {
    return static_cast<const T&>(*this);
  }

private:
  const Type type_;
};
}

#endif //DMP_SHAPE_H
