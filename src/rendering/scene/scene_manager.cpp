#include <dmp/rendering/scene/scene_manager.h>

namespace dmp
{
SceneManager::SceneManager()
{
}

std::shared_ptr<SceneNode> SceneManager::createNode()
{
  return std::make_shared<SceneNode>();
}
}