#ifndef DMP_SCENE_MESH_OBJECT_H
#define DMP_SCENE_MESH_OBJECT_H

#include <dmp/rendering/scene/scene_object.h>

#include <string>

namespace dmp
{
class SceneMeshObject : public SceneObject
{
public:
  explicit SceneMeshObject(const std::string& filename);

private:
  std::string filename_;
};
}

#endif //DMP_SCENE_MESH_OBJECT_H
