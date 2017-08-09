#include <dmp/rendering/tf/tf_manager.h>
#include <dmp/rendering/tf/tf.h>

namespace dmp
{
TfManager::TfManager()
    : origin_("origin")
{
}

void TfManager::setOrigin(const std::string& origin)
{
  origin_ = origin;
}

void TfManager::set(const std::string& from, const std::string& to, const Eigen::Affine3d& transform)
{
  std::shared_ptr<Tf> tf_from;
  std::shared_ptr<Tf> tf_to;

  if (tf_map_.find(from) == tf_map_.cend())
  {
    tf_from = std::make_shared<Tf>(from);
    tf_map_[from] = tf_from;
  }
  else
    tf_from = tf_map_[from];

  if (tf_map_.find(to) == tf_map_.cend())
  {
    tf_to = std::make_shared<Tf>(to);
    tf_map_[to] = tf_to;
  }
  else
    tf_to = tf_map_[to];

  tf_to->setParent(tf_from, transform);
}
}
