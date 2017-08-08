#include <dmp/rendering/scene/scene_manager.h>
#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_object.h>
#include <dmp/rendering/scene/scene_mesh_object.h>

namespace dmp
{
std::shared_ptr<SceneNode> SceneManager::createNode()
{
  return std::make_shared<SceneNode>();
}

std::shared_ptr<SceneNode> SceneManager::createNode(const Eigen::Affine3d& transform)
{
  return std::make_shared<SceneNode>(transform);
}

std::shared_ptr<SceneNode> SceneManager::createNode(const Eigen::Vector3d& translate)
{
  return std::make_shared<SceneNode>(translate);
}

std::shared_ptr<SceneObject> SceneManager::createMeshObject(const std::string& filename)
{
  return std::make_shared<SceneMeshObject>(filename);
}
}