#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/scene/scene_object.h>
#include <dmp/rendering/scene/scene_mesh_object.h>
#include <thread>

namespace dmp
{
SceneManager::SceneManager()
    : root_(std::make_shared<SceneNode>())
{
}

std::shared_ptr<SceneNode> SceneManager::createNode()
{
  std::lock_guard<std::mutex> lock{mutex_};

  auto node = std::make_shared<SceneNode>();
  root_->createEdge(node);
  return node;
}

std::shared_ptr<SceneObject> SceneManager::createMeshObject(const std::string& filename)
{
  return std::make_shared<SceneMeshObject>(filename);
}

std::shared_ptr<SceneNode> SceneManager::getRoot()
{
  return root_;
}
}