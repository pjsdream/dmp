#ifndef DMP_SCENE_NODE_H
#define DMP_SCENE_NODE_H

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace dmp
{
class SceneEdge;
class Resource;
class SceneNode : public std::enable_shared_from_this<SceneNode>
{
public:
  SceneNode() = default;
  explicit SceneNode(const std::string& name);
  ~SceneNode() = default;

  SceneNode(const SceneNode& rhs) = delete;
  SceneNode& operator=(const SceneNode& rhs) = delete;

  SceneNode(SceneNode&& rhs) = default;
  SceneNode& operator=(SceneNode&& rhs) = default;

  void attachResource(const std::shared_ptr<Resource>& resource);

  void setTransform(const Eigen::Affine3d& transform);
  const Eigen::Affine3d& getTransform();
  const std::vector<SceneEdge>& getEdges();
  const std::vector<std::shared_ptr<Resource>>& getAttachedResources();

private:
  std::string name_;

  Eigen::Affine3d transform_;

  std::vector<std::shared_ptr<Resource>> resources_;
};
}

#endif //DMP_SCENE_NODE_H
