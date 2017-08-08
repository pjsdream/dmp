#ifndef DMP_SCENE_NODE_H
#define DMP_SCENE_NODE_H

#include <Eigen/Dense>

#include <memory>

namespace dmp
{
class SceneObject;
class SceneNode
{
public:
  SceneNode();
  explicit SceneNode(const Eigen::Affine3d& transform);
  explicit SceneNode(const Eigen::Vector3d& translate);
  ~SceneNode() = default;

  SceneNode(const SceneNode& rhs) = delete;
  SceneNode& operator=(const SceneNode& rhs) = delete;

  SceneNode(SceneNode&& rhs) = default;
  SceneNode& operator=(SceneNode&& rhs) = default;

  void attachObject(const std::shared_ptr<SceneObject>& object);

  const std::vector<std::shared_ptr<SceneObject>>& getAttachedObjects();

private:
  Eigen::Affine3d transform_;

  std::vector<std::shared_ptr<SceneObject>> objects_;
};
}

#endif //DMP_SCENE_NODE_H
