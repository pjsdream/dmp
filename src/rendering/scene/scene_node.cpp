#include <dmp/rendering/scene/scene_node.h>
#include <dmp/rendering/scene/scene_edge.h>
#include <dmp/rendering/resource/resource.h>

namespace dmp
{
SceneNode::SceneNode(const std::string& name)
    : name_(name)
{
}

void SceneNode::attachResource(const std::shared_ptr<Resource>& resource)
{
  resources_.push_back(resource);
}

const std::vector<SceneEdge>& SceneNode::getEdges()
{
  return edges_;
}

const std::vector<std::shared_ptr<Resource>>& SceneNode::getAttachedResources()
{
  return resources_;
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
