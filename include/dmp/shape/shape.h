#ifndef DMP_SHAPE_H
#define DMP_SHAPE_H

#include <Eigen/Dense>

namespace dmp
{
class Shape
{
public:
  Shape();
  virtual ~Shape();

  void setTransform(const Eigen::Affine3d& transform);

  const Eigen::Affine3d& getTransform();

private:
  Eigen::Affine3d transform_;
};
}

#endif //DMP_SHAPE_H
