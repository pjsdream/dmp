#ifndef DMP_SCENE_MANAGER_H
#define DMP_SCENE_MANAGER_H

#include <memory>

#include <Eigen/Dense>

namespace dmp
{
class SceneNode;
class SceneObject;
class SceneManager
{
public:
  SceneManager() = default;
  ~SceneManager() = default;

  SceneManager(const SceneManager& rhs) = delete;
  SceneManager& operator=(const SceneManager& rhs) = delete;

  SceneManager(SceneManager&& rhs) = delete;
  SceneManager& operator=(SceneManager&& rhs) = delete;

  std::shared_ptr<SceneNode> createNode();
  std::shared_ptr<SceneNode> createNode(const Eigen::Affine3d& transform);
  std::shared_ptr<SceneNode> createNode(const Eigen::Vector3d& translate);

  std::shared_ptr<SceneObject> createMeshObject(const std::string& filename);

private:
};
}

#endif //DMP_SCENE_MANAGER_H
