#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/resource/resource.h>
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

std::shared_ptr<SceneNode> SceneManager::getNode(const std::string& name)
{
  if (node_map_.find(name) == node_map_.cend())
    return nullptr;
  else
    return node_map_[name];
}

void SceneManager::attachResource(const std::string& name, std::shared_ptr<Resource> resource)
{
  if (node_map_.find(name) != node_map_.cend())
    node_map_[name]->attachResource(resource);
}

std::vector<std::shared_ptr<SceneNode>> SceneManager::traverseNodes()
{
  std::vector<std::shared_ptr<SceneNode>> result;
  traverseNodes(result);
  return result;
}

void SceneManager::traverseNodes(std::vector<std::shared_ptr<SceneNode>>& result)
{
}
}