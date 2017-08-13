#include <dmp/rendering/request/request_light.h>

namespace dmp
{
RequestLight::RequestLight()
    : action_{Action::Nothing}
{
}

RequestLight::~RequestLight() = default;

void RequestLight::setLight(int index, Light&& light)
{
  action_ = Action::Set;
  index_ = index;
  light_ = std::move(light);
}

void RequestLight::deleteLight(int index)
{
  action_ = Action::Delete;
}

RequestLight::Action RequestLight::getAction()
{
  return action_;
}

int RequestLight::getIndex()
{
  return index_;
}

const Light& RequestLight::getLight()
{
  return light_;
}
}