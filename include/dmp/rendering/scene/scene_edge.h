#ifndef DMP_SCENE_EDGE_H
#define DMP_SCENE_EDGE_H

#include <Eigen/Dense>

namespace dmp
{
class SceneNode;
class SceneEdge
{
public:
  SceneEdge() = delete;
  explicit SceneEdge(const std::shared_ptr<SceneNode>& child);
  explicit SceneEdge(const std::shared_ptr<SceneNode>& child, const Eigen::Affine3d& transform);

  std::shared_ptr<SceneNode> getChild() const noexcept;
  const Eigen::Affine3d& getTransform() const noexcept;

private:
  std::shared_ptr<SceneNode> child_;
  Eigen::Affine3d transform_;
};
}

#endif //DMP_SCENE_EDGE_H
