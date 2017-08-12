#include <dmp/rendering/resource/resource_manager.h>
#include <dmp/rendering/resource/resource_mesh.h>

namespace dmp
{
ResourceManager::ResourceManager(const std::shared_ptr<GlFunctions>& gl)
    : gl_{gl}
{
}

std::shared_ptr<ResourceMesh> ResourceManager::getMesh(const std::string& filename)
{
  auto it = meshes_.find(filename);
  if (it != meshes_.cend())
    return it->second;

  auto mesh = std::make_shared<ResourceMesh>(gl_, filename);
  meshes_[filename] = mesh;
  return mesh;
}
}
