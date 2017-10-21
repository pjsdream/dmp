#include <dmp/planning/environment/interactable_object.h>

namespace dmp
{
InteractableObject::InteractableObject()
    : grip_transform_(Eigen::Affine3d::Identity())
{
}

void InteractableObject::setGripTransform(const Eigen::Affine3d& transform) noexcept
{
  grip_transform_ = transform;
}

const Eigen::Affine3d& InteractableObject::getGripTransform() const noexcept
{
  return grip_transform_;
}
}
