#ifndef DMP_SCENE_OBJECT_H
#define DMP_SCENE_OBJECT_H

namespace dmp
{
class SceneObject
{
public:
  SceneObject() = default;
  virtual ~SceneObject() = default;

  SceneObject(const SceneObject& rhs) = default;
  SceneObject& operator = (const SceneObject& rhs) = default;

  SceneObject(SceneObject&& rhs) = default;
  SceneObject& operator = (SceneObject&& rhs) = default;

  virtual bool isMeshObject() const noexcept;

private:
};
}
#endif //DMP_SCENE_OBJECT_H
