#include <dmp/rendering/scene/scene_node.h>

namespace dmp
{
SceneNode::SceneNode()
    : transform_(Eigen::Affine3d::Identity())
{
}

SceneNode::SceneNode(const Eigen::Affine3d& transform)
    : transform_(transform)
{
}

SceneNode::SceneNode(const Eigen::Vector3d& translate)
    : transform_(Eigen::Affine3d::Identity())
{
  transform_.translate(translate);
}

void SceneNode::attachObject(const std::shared_ptr<SceneObject>& object)
{
  objects_.push_back(object);
}

const std::vector<std::shared_ptr<SceneObject>>& SceneNode::getAttachedObjects()
{
  return objects_;
}
}