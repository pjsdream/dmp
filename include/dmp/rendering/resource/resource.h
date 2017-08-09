#ifndef DMP_RESOURCE_H
#define DMP_RESOURCE_H

#include <dmp/rendering/gl_base.h>

namespace dmp
{
class Resource
{
public:
  Resource() = delete;
  explicit Resource(const std::shared_ptr<GlFunctions>& gl);
  virtual ~Resource() = default;

protected:
  std::shared_ptr<GlFunctions> gl_;
};
}

#endif //DMP_RESOURCE_H
