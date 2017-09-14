#ifndef DMP_OBJECT_H
#define DMP_OBJECT_H

#include <Eigen/Dense>

#include <memory>

namespace dmp
{
class Shape;
class Object
{
public:
  void setShape(const std::shared_ptr<Shape>& shape);
  const std::shared_ptr<Shape>& getShape() const;

  void setColor(const Eigen::Vector4f& color);
  const Eigen::Vector4f& getColor() const;

private:
  std::shared_ptr<Shape> shape_;
  Eigen::Vector4f color_;
};
}

#endif //DMP_OBJECT_H
