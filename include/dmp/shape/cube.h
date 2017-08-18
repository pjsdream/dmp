#ifndef DMP_CUBE_H
#define DMP_CUBE_H

#include <dmp/shape/shape.h>

namespace dmp
{
class Cube : public Shape
{
public:
  Cube();
  ~Cube() override;

  void setSize(const Eigen::Vector3d& size);

  const Eigen::Vector3d& getSize();

private:
  Eigen::Vector3d size_;
};
}

#endif //DMP_CUBE_H
