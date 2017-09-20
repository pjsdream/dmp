#include <dmp/robot/robot_link.h>
#include <dmp/robot/robot_joint.h>

namespace dmp
{
void RobotLink::setName(const std::string& name)
{
  name_ = name;
}

const std::string& RobotLink::getName() const
{
  return name_;
}

void RobotLink::setParentJoint(const std::shared_ptr<RobotJoint>& parent)
{
  parent_ = parent;
}

void RobotLink::addChildJoint(const std::shared_ptr<RobotJoint>& child)
{
  children_.push_back(child);
}

const std::vector<std::shared_ptr<RobotJoint>>& RobotLink::getChildJoints() const
{
  return children_;
}

void RobotLink::addVisualMesh(const std::string& filename, const Eigen::Affine3d& transform)
{
  Visual visual;
  visual.filename = filename;
  visual.transform = transform;
  visual.has_color = false;

  visuals_.push_back(visual);
}

void RobotLink::addVisualMesh(const std::string& filename,
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

const std::vector<RobotLink::Visual>& RobotLink::getVisuals() const noexcept
{
  return visuals_;
}

void RobotLink::addCollisionMesh(const std::string& filename, const Eigen::Affine3d& transform) noexcept
{
  Collision collision;
  collision.filename = filename;
  collision.transform = transform;

  collisions_.push_back(collision);
}

const std::vector<RobotLink::Collision>& RobotLink::getCollisions() const noexcept
{
  return collisions_;
}
}
