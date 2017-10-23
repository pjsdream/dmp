#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/scene/scene_node.h>

namespace dmp
{
SceneEdge::SceneEdge(const std::shared_ptr<SceneNode>& child)
    : child_(child), transform_(Eigen::Affine3d::Identity())
{
}

SceneEdge::SceneEdge(const std::shared_ptr<SceneNode>& child, const Eigen::Affine3d& transform)
    : child_(child), transform_(transform)
{
}

std::shared_ptr<SceneNode> SceneEdge::getChild() const noexcept
{
  return child_;
}

const Eigen::Affine3d& SceneEdge::getTransform() const noexcept
{
  return transform_;
}
}
