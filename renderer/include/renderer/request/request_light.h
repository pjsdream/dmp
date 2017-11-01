#ifndef DMP_REQUEST_LIGHT_H
#define DMP_REQUEST_LIGHT_H

#include "request.h"
#include <renderer/light/light.h>

#include <Eigen/Dense>

namespace dmp
{
class RequestLight : public Request
{
public:
  enum class Action : unsigned char
  {
    Nothing,
    Set,
    Delete,
  };

  enum class LightType : unsigned char
  {
    Directional,
    Point
  };

  RequestLight()
      : Request(Request::Type::Light)
  {
  }

  ~RequestLight() override = default;

  void setLight(int index, const Light& light)
  {
    this->index = index;
    action = Action::Set;

    // TODO: store light
    switch (light.type)
    {
      case Light::LightType::Directional:
        light_type = LightType::Directional;
        break;

      case Light::LightType::Point:
        light_type = LightType::Point;
        break;
    }

    position = light.position;
    ambient = light.ambient;
    diffuse = light.diffuse;
    specular = light.specular;
    attenuation = light.attenuation;
  }

  void deleteLight(int index)
  {
    index = index;
    action = Action::Delete;
  }

  Light getLight()
  {
    Light light;

    switch (light_type)
    {
      case LightType::Directional:
        light.type = Light::LightType::Directional;
        break;

      case LightType::Point:
        light.type = Light::LightType::Point;
        break;
    }

    light.position = position;
    light.ambient = ambient;
    light.diffuse = diffuse;
    light.specular = specular;
    light.attenuation = attenuation;

    return light;
  }

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    // TODO: serialize light
    return ar & action & index & light_type & position & ambient & diffuse & specular & attenuation;
  }

  Action action = Action::Nothing;
  int index = 0;
  LightType light_type = LightType::Directional;
  Eigen::Vector3f position;
  Eigen::Vector3f ambient;
  Eigen::Vector3f diffuse;
  Eigen::Vector3f specular;
  Eigen::Vector3f attenuation;
};
}

#endif //DMP_REQUEST_LIGHT_H
