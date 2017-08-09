#ifndef DMP_SCENE_NODE_H
#define DMP_SCENE_NODE_H

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace dmp
{
class SceneEdge;
class SceneObject;
class SceneNode : public std::enable_shared_from_this<SceneNode>
{
public:
  SceneNode();
  ~SceneNode() = default;

  SceneNode(const SceneNode& rhs) = delete;
  SceneNode& operator=(const SceneNode& rhs) = delete;

  SceneNode(SceneNode&& rhs) = default;
  SceneNode& operator=(SceneNode&& rhs) = default;

  void attachObject(const std::shared_ptr<SceneObject>& object);

  void createEdge(const std::shared_ptr<SceneNode>& child);
  void createEdge(const std::shared_ptr<SceneNode>& child, const Eigen::Affine3d& transform);

  const Eigen::Affine3d& getTransform();
  const std::vector<SceneEdge>& getEdges();
  const std::vector<std::shared_ptr<SceneObject>>& getAttachedObjects();

private:
  std::vector<SceneEdge> edges_;

  std::vector<std::shared_ptr<SceneObject>> objects_;
};
}

#endif //DMP_SCENE_NODE_H
