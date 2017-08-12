#ifndef DMP_LIGHT_H
#define DMP_LIGHT_H

#include <Eigen/Dense>

namespace dmp
{
struct Light
{
  enum class LightType
  {
    Directional,
    Point
  };

  LightType type;
  Eigen::Vector3f position;
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
};
}

#endif //DMP_LIGHT_H
