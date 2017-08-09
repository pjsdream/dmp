#ifndef DMP_TF_H
#define DMP_TF_H

#include <string>
#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace dmp
{
class Tf : public std::enable_shared_from_this<Tf>
{
public:
  Tf() = delete;
  explicit Tf(const std::string& name);
  ~Tf() = default;

  void setParent(const std::shared_ptr<Tf>& parent, const Eigen::Affine3d& transform);
  void setTransform(const Eigen::Affine3d& transform);

private:
  std::string name_;
  Eigen::Affine3d transform_from_parent_;

  std::weak_ptr<Tf> parent_;
  std::vector<std::shared_ptr<Tf>> children_;
};
}

#endif //DMP_TF_H
