#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>

namespace dmp
{
SceneNode::SceneNode()
{
}

void SceneNode::attachObject(const std::shared_ptr<SceneObject>& object)
{
  objects_.push_back(object);
}

const std::vector<SceneEdge>& SceneNode::getEdges()
{
  return edges_;
}

const std::vector<std::shared_ptr<SceneObject>>& SceneNode::getAttachedObjects()
{
  return objects_;
}

void SceneNode::createEdge(const std::shared_ptr<SceneNode>& child)
{
  edges_.emplace_back(child);
}

void SceneNode::createEdge(const std::shared_ptr<SceneNode>& child, const Eigen::Affine3d& transform)
{
  edges_.emplace_back(child, transform);
}
}
