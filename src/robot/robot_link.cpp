#include <dmp/robot/robot_link.h>

namespace dmp
{
RobotLink::RobotLink(const std::string& name)
    : name_(name)
{
}

const std::string& RobotLink::getName() const noexcept
{
  return name_;
}

void RobotLink::addVisual(const std::string& filename, const Eigen::Affine3d& transform)
{
  Visual visual;
  visual.filename = filename;
  visual.transform = transform;
  visual.has_color = false;

  visuals_.emplace_back(visual);
}

void RobotLink::addVisual(const std::string& filename,
                          const Eigen::Affine3d& transform,
                          bool has_color,
                          const Eigen::Vector4d& color)
{
  Visual visual;
  visual.filename = filename;
  visual.transform = transform;
  visual.has_color = has_color;
  visual.color = color;

  visuals_.emplace_back(visual);
}

const std::vector<RobotLink::Visual> RobotLink::getVisuals() const noexcept
{
  return visuals_;
}

void RobotLink::addCollision(const std::string& filename, const Eigen::Affine3d& transform)
{
  Collision collision;
  collision.filename = filename;
  collision.transform = transform;

  collisions_.emplace_back(collision);
}
}
