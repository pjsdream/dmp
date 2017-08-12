#ifndef DMP_LIGHT_MANAGER_H
#define DMP_LIGHT_MANAGER_H

#include <memory>
#include <unordered_map>

namespace dmp
{
class Light;
class LightManager
{
public:
  LightManager() = default;
  ~LightManager() = default;

  std::shared_ptr<Light> setLight(int index, const Light& light);
  void deleteLight(int index);

  const std::unordered_map<int, std::shared_ptr<Light>>& getLights();

private:
  std::shared_ptr<Light> getLight(int index);

  std::unordered_map<int, std::shared_ptr<Light>> lights_;
};
}

#endif //DMP_LIGHT_MANAGER_H
