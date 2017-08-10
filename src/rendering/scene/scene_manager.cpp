#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <thread>

namespace dmp
{
SceneManager::SceneManager()
    : root_(std::make_shared<SceneNode>())
{
}

SceneManager::~SceneManager() = default;

std::shared_ptr<SceneNode> SceneManager::createNode(const std::string& name)
{
  if (node_map_.find(name) == node_map_.cend())
  {
    auto node = std::make_shared<SceneNode>(name);
    node_map_[name] = node;
    return node;
  }
  else
    return node_map_[name];
}

void SceneManager::deleteNode(const std::string& name)
{
  if (node_map_.find(name) != node_map_.cend())
  {
    // TODO: delete edge from parent to the node
    auto node = node_map_[name];

    node_map_.erase(name);
  }
}

std::shared_ptr<SceneNode> SceneManager::getRoot()
{
  return root_;
}

std::shared_ptr<SceneNode> SceneManager::getNode(const std::string& name)
{
  if (node_map_.find(name) == node_map_.cend())
    return nullptr;
  else
    return node_map_[name];
}
}