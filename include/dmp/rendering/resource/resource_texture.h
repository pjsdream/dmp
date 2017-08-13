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

  void loadTexture(const std::string& filename);
  void bind();

private:
  GLuint texture_id_;
};
}

#endif //DMP_RESOURCE_TEXTURE_H
