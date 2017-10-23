#ifndef DMP_MATERIAL_H
#define DMP_MATERIAL_H

#include <Eigen/Dense>

namespace dmp
{
struct Material
{
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  float shininess;
};
}

#endif //DMP_MATERIAL_H
