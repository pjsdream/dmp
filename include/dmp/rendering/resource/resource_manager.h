#ifndef DMP_RESOURCE_MANAGER_H
#define DMP_RESOURCE_MANAGER_H

#include <dmp/rendering/gl_base.h>

#include <unordered_map>
#include <memory>

namespace dmp
{
class ResourceMesh;
class ResourceTexture;
class MeshLoaderRawMesh;
class TextureLoaderRawTexture;
class ResourceManager
{
public:
  ResourceManager() = delete;
  explicit ResourceManager(const std::shared_ptr<GlFunctions>& gl);
  ~ResourceManager() = default;

  ResourceManager(const ResourceManager& rhs) = delete;
  ResourceManager& operator=(const ResourceManager& rhs) = delete;

  ResourceManager(ResourceManager&& rhs) = delete;
  ResourceManager& operator=(ResourceManager&& rhs) = delete;

  std::shared_ptr<ResourceMesh> createMesh(const std::string& name, MeshLoaderRawMesh&& raw_mesh);
  std::shared_ptr<ResourceMesh> getMesh(const std::string& filename);
  std::shared_ptr<ResourceTexture> createTexture(const std::string& name, TextureLoaderRawTexture&& raw_texture);
  std::shared_ptr<ResourceTexture> getTexture(const std::string& name);

private:
  static const int MAX_NUM_LIGHTS = 8;

  std::shared_ptr<GlFunctions> gl_;

  std::unordered_map<std::string, std::shared_ptr<ResourceMesh>> meshes_;
  std::unordered_map<std::string, std::shared_ptr<ResourceTexture>> textures_;
};
}

#endif //DMP_RESOURCE_MANAGER_H
