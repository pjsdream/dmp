#ifndef DMP_TF_MANAGER_H
#define DMP_TF_MANAGER_H

#include <string>
#include <memory>
#include <unordered_map>

#include <Eigen/Dense>

namespace dmp
{
class Tf;
class TfManager
{
public:
  TfManager();
  ~TfManager() = default;

  void setRoot(const std::string& origin);
  void set(const std::string& from, const std::string& to, const Eigen::Affine3d& transform);

private:
  std::string origin_;
  std::unordered_map<std::string, std::shared_ptr<Tf>> tf_map_;
};
}

#endif //DMP_TF_MANAGER_H
