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

private:
};
}
#endif //DMP_SCENE_OBJECT_H
