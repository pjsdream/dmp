#include <dmp/rendering/resource/resource_manager.h>
#include <dmp/rendering/resource/resource_mesh.h>
#include <dmp/rendering/resource/resource_texture.h>

#include <dmp/utils/texture_loader.h>

namespace dmp
{
ResourceManager::ResourceManager(const std::shared_ptr<GlFunctions>& gl)
    : gl_{gl}
{
}

std::shared_ptr<ResourceMesh> ResourceManager::createMesh(const std::string& name, MeshLoaderRawMesh&& raw_mesh)
{
  auto mesh = std::make_shared<ResourceMesh>(gl_);
  mesh->loadMesh(std::move(raw_mesh));
  meshes_[name] = mesh;
  return mesh;
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

std::shared_ptr<ResourceTexture> ResourceManager::createTexture(const std::string& name,
                                                                TextureLoaderRawTexture&& raw_texture)
{
  auto texture = std::make_shared<ResourceTexture>(gl_);
  texture->loadTexture(std::move(raw_texture));
  textures_[name] = texture;
  return texture;
}

std::shared_ptr<ResourceTexture> ResourceManager::getTexture(const std::string& name)
{
  if (textures_.find(name) == textures_.cend())
    return nullptr;
  return textures_[name];
}
}
