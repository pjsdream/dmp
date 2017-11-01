#include <renderer/resource/resource_manager.h>
#include <renderer/resource/resource_mesh.h>
#include <renderer/resource/resource_texture.h>
#include <renderer/resource/texture_loader.h>

namespace dmp
{
ResourceManager::ResourceManager(const std::shared_ptr<GlFunctions>& gl)
    : gl_{gl}
{
}

std::shared_ptr<ResourceMesh> ResourceManager::createMesh(const std::string& name, RawMesh&& raw_mesh)
{
  auto mesh = std::make_shared<ResourceMesh>(gl_);
  mesh->loadMesh(std::move(raw_mesh));
  meshes_[name] = mesh;
  return mesh;
}

std::shared_ptr<ResourceMesh> ResourceManager::loadMesh(const std::string& name, const std::string& filename)
{
  auto it = mesh_name_to_filename_.find(name);
  if (it != mesh_name_to_filename_.cend() && it->second == filename)
    return meshes_[name];

  mesh_name_to_filename_[name] = filename;
  auto mesh = std::make_shared<ResourceMesh>(gl_, filename);
  meshes_[name] = mesh;
  return mesh;
}

std::shared_ptr<ResourceMesh> ResourceManager::getMesh(const std::string& name)
{
  auto it = meshes_.find(name);
  if (it != meshes_.cend())
    return it->second;
  return nullptr;
}

std::shared_ptr<ResourceTexture> ResourceManager::createTexture(const std::string& name,
                                                                TextureLoaderRawTexture&& raw_texture)
{
  auto texture = std::make_shared<ResourceTexture>(gl_);
  texture->loadTexture(std::move(raw_texture));
  textures_[name] = texture;
  return texture;
}

std::shared_ptr<ResourceTexture> ResourceManager::getTexture(const std::string& name, const std::string& filename)
{
  auto it = texture_name_to_filename_.find(name);
  if (textures_.find(name) != textures_.cend() && it->second == filename)
    return textures_[name];

  // TODO: load texture from file
  return nullptr;
}
}
