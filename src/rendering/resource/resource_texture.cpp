#include <dmp/rendering/resource/resource_texture.h>

namespace dmp
{
ResourceTexture::ResourceTexture(const std::shared_ptr<GlFunctions>& gl)
    : Resource(gl_)
{
}

ResourceTexture::~ResourceTexture()
{
  // TODO: delete gl resources
}
}
