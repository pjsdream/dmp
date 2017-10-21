#include <dmp/rendering/light/light_manager.h>
#include <dmp/rendering/light/light.h>

namespace dmp
{
void LightManager::setLight(int index, const Light& light)
{
  *getLight(index) = light;
}

void LightManager::deleteLight(int index)
{
  lights_.erase(index);
}

const std::unordered_map<int, std::shared_ptr<Light>>& LightManager::getLights()
{
  return lights_;
}

std::shared_ptr<Light> LightManager::getLight(int index)
{
  if (lights_.find(index) == lights_.cend())
    lights_[index] = std::make_shared<Light>();
  return lights_[index];
}
}
