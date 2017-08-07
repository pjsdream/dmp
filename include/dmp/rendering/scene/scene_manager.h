#ifndef DMP_SCENE_MANAGER_H
#define DMP_SCENE_MANAGER_H

#include <memory>

namespace dmp
{
class SceneNode;
class SceneManager
{
public:
  SceneManager();
  ~SceneManager() = default;

  std::shared_ptr<SceneNode> createNode();

private:
};
}

#endif //DMP_SCENE_MANAGER_H
