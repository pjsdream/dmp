#ifndef DMP_RESOURCE_H
#define DMP_RESOURCE_H

#include <renderer/gl_base.h>

#include <memory>

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
