#ifndef DMP_AABB_H
#define DMP_AABB_H

#include <planner/shape/shape.h>

namespace dmp
{
class AABB : public Shape
{
public:
  AABB() = delete;
  explicit AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max) noexcept;

  const Eigen::Vector3d& getMin() const noexcept;
  const Eigen::Vector3d& getMax() const noexcept;

private:
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
};
}

#endif //DMP_AABB_H
