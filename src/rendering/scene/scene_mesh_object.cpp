#include <dmp/rendering/scene/scene_mesh_object.h>

namespace dmp
{
SceneMeshObject::SceneMeshObject(const std::string& filename)
    : filename_(filename)
{
}

bool SceneMeshObject::isMeshObject() const noexcept
{
  return true;
}

std::string SceneMeshObject::getFilename() const
{
  return filename_;
}
}
