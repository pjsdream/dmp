#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/resource/resource.h>
#include <thread>

namespace dmp
{
SceneManager::SceneManager()
{
  createNode("");
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

void SceneManager::setFrame(const std::string& name, const Eigen::Affine3d& transform)
{
  setFrame("", name, transform);
}

void SceneManager::setFrame(const std::string& from, const std::string& to, const Eigen::Affine3d& transform)
{
  edges_[from][to] = transform;
}

void SceneManager::attachResource(const std::string& name, std::shared_ptr<Resource> resource)
{
  createNode(name)->attachResource(resource);
}

std::vector<std::shared_ptr<SceneNode>> SceneManager::traverseNodes()
{
  std::vector<std::shared_ptr<SceneNode>> result;
  traverseNodes(result, "");
  return result;
}

void SceneManager::traverseNodes(std::vector<std::shared_ptr<SceneNode>>& result, const std::string& name)
{
  auto node = getNode(name);
  auto transform = node->getTransform();

  result.push_back(node);

  for (auto edge : edges_[name])
  {
    auto to = edge.first;
    auto edge_transform = edge.second;

    createNode(to)->setTransform(edge_transform);

    traverseNodes(result, to);
  }
}
}