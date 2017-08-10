#ifndef DMP_RESOURCE_TEXTURE_H
#define DMP_RESOURCE_TEXTURE_H

#include <dmp/rendering/resource/resource.h>

namespace dmp
{
class ResourceTexture : public Resource
{
public:
  ResourceTexture() = delete;
  ResourceTexture(const std::shared_ptr<GlFunctions>& gl);
  ~ResourceTexture() override;

private:
};
}

#endif //DMP_RESOURCE_TEXTURE_H
