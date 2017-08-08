#ifndef DMP_RESOURCE_MANAGER_H
#define DMP_RESOURCE_MANAGER_H

#include <dmp/rendering/gl_base.h>

#include <unordered_map>

namespace dmp
{
class ResourceMesh;

class ResourceManager : public GlBase
{
public:
  ResourceManager() = default;
  ~ResourceManager() override = default;

  ResourceManager(const ResourceManager& rhs) = delete;
  ResourceManager& operator=(const ResourceManager& rhs) = delete;

  ResourceManager(ResourceManager&& rhs) = delete;
  ResourceManager& operator=(ResourceManager&& rhs) = delete;

  std::shared_ptr<ResourceMesh> getMesh(const std::string& filename);

private:
  std::unordered_map<std::string, std::shared_ptr<ResourceMesh>> meshes_;
};
}

#endif //DMP_RESOURCE_MANAGER_H
