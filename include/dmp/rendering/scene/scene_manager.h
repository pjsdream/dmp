#ifndef DMP_SCENE_MANAGER_H
#define DMP_SCENE_MANAGER_H

#include <memory>
#include <mutex>
#include <unordered_map>

#include <Eigen/Dense>

namespace dmp
{
class SceneNode;
class SceneObject;
class SceneManager
{
public:
  SceneManager();
  ~SceneManager();

  SceneManager(const SceneManager& rhs) = delete;
  SceneManager& operator=(const SceneManager& rhs) = delete;

  SceneManager(SceneManager&& rhs) = delete;
  SceneManager& operator=(SceneManager&& rhs) = delete;

  std::shared_ptr<SceneNode> createNode(const std::string& name);
  void deleteNode(const std::string& name);

  std::shared_ptr<SceneNode> getRoot();
  std::shared_ptr<SceneNode> getNode(const std::string& name);

private:
  std::shared_ptr<SceneNode> root_;
  std::unordered_map<std::string, std::shared_ptr<SceneNode>> node_map_;
};
}

#endif //DMP_SCENE_MANAGER_H
