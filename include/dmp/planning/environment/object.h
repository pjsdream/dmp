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
  Object();
  virtual ~Object() = default;

  void setShape(const std::shared_ptr<Shape>& shape);
  const std::shared_ptr<Shape>& getShape() const;

  void setColor(const Eigen::Vector4f& color);
  const Eigen::Vector4f& getColor() const;

  void setName(const std::string& name) noexcept;
  const std::string& getName() const noexcept;

  void setTransform(const Eigen::Affine3d& transform) noexcept;
  const Eigen::Affine3d& getTransform() const noexcept;

private:
  std::string name_;
  std::shared_ptr<Shape> shape_;
  Eigen::Vector4f color_;
  Eigen::Affine3d transform_;
};
}

#endif //DMP_OBJECT_H
