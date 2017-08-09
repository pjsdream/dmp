#ifndef DMP_SCENE_MANAGER_H
#define DMP_SCENE_MANAGER_H

#include <memory>
#include <mutex>

#include <Eigen/Dense>

namespace dmp
{
class SceneNode;
class SceneObject;
class SceneManager
{
public:
  SceneManager();
  ~SceneManager() = default;

  SceneManager(const SceneManager& rhs) = delete;
  SceneManager& operator=(const SceneManager& rhs) = delete;

  SceneManager(SceneManager&& rhs) = delete;
  SceneManager& operator=(SceneManager&& rhs) = delete;

  std::shared_ptr<SceneNode> createNode();

  std::shared_ptr<SceneObject> createMeshObject(const std::string& filename);

  std::shared_ptr<SceneNode> getRoot();

private:
  std::mutex mutex_;

  std::shared_ptr<SceneNode> root_;
};
}

#endif //DMP_SCENE_MANAGER_H
