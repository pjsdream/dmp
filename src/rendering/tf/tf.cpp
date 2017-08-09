#include <dmp/rendering/tf/tf.h>

namespace dmp
{
Tf::Tf(const std::string& name)
: name_(name)
{
}

void Tf::setParent(const std::shared_ptr<Tf>& parent, const Eigen::Affine3d& transform)
{
  if (parent_.lock() != nullptr)
  {
    // TODO: remove this from parent's children
  }

  parent_ = parent;
  parent->children_.push_back(shared_from_this());

  transform_from_parent_ = transform;
}

void Tf::setTransform(const Eigen::Affine3d& transform)
{
  transform_from_parent_ = transform;
}
}
