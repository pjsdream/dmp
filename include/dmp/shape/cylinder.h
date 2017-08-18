#ifndef DMP_CYLINDER_H
#define DMP_CYLINDER_H

#include <dmp/shape/shape.h>

namespace dmp
{
class Cylinder : public Shape
{
public:
  Cylinder();
  ~Cylinder() override;

  void setDimension(double radius, double height);

  double getRadius();
  double getHeight();

private:
  double radius_;
  double height_;
};
}

#endif //DMP_CYLINDER_H
