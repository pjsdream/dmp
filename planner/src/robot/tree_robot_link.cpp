#include <dmp/robot/tree_robot_link.h>
#include <dmp/robot/tree_robot_joint.h>

namespace dmp
{
void TreeRobotLink::setName(const std::string& name)
{
  name_ = name;
}

const std::string& TreeRobotLink::getName() const
{
  return name_;
}

void TreeRobotLink::setParentJoint(const std::shared_ptr<TreeRobotJoint>& parent)
{
  parent_ = parent;
}

void TreeRobotLink::addChildJoint(const std::shared_ptr<TreeRobotJoint>& child)
{
  children_.push_back(child);
}

const std::vector<std::shared_ptr<TreeRobotJoint>>& TreeRobotLink::getChildJoints() const
{
  return children_;
}

void TreeRobotLink::addVisualMesh(const std::string& filename, const Eigen::Affine3d& transform)
{
  Visual visual;
  visual.filename = filename;
  visual.transform = transform;
  visual.has_color = false;

  visuals_.push_back(visual);
}

void TreeRobotLink::addVisualMesh(const std::string& filename,
                              const Eigen::Affine3d& transform,
                              const Eigen::Vector4d& color)
{
  Visual visual;
  visual.filename = filename;
  visual.transform = transform;
  visual.has_color = true;
  visual.color = color;

  visuals_.push_back(visual);
}

const std::vector<TreeRobotLink::Visual>& TreeRobotLink::getVisuals() const noexcept
{
  return visuals_;
}

void TreeRobotLink::addCollisionMesh(const std::string& filename, const Eigen::Affine3d& transform) noexcept
{
  Collision collision;
  collision.filename = filename;
  collision.transform = transform;

  collisions_.push_back(collision);
}

const std::vector<TreeRobotLink::Collision>& TreeRobotLink::getCollisions() const noexcept
{
  return collisions_;
}
}
