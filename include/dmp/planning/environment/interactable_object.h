#ifndef DMP_INTERACTABLE_OBJECT_H
#define DMP_INTERACTABLE_OBJECT_H

#include <dmp/planning/environment/object.h>

namespace dmp
{
class InteractableObject : public Object
{
public:
  InteractableObject();
  ~InteractableObject() override = default;

  void setGripTransform(const Eigen::Affine3d& transform) noexcept;
  const Eigen::Affine3d& getGripTransform() const noexcept;

private:
  // Have only one gripping position/orientation. TODO: have multiple.
  Eigen::Affine3d grip_transform_;
};
}
#endif //DMP_INTERACTABLE_OBJECT_H
