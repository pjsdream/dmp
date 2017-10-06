#include <dmp/rendering/resource/resource_texture.h>
#include <dmp/utils/texture_loader.h>

namespace dmp
{
ResourceTexture::ResourceTexture(const std::shared_ptr<GlFunctions>& gl)
    : Resource(gl)
{
}

ResourceTexture::~ResourceTexture()
{
  printf("deleting texture\n");
  gl_->glDeleteTextures(1, &texture_id_);
}

void ResourceTexture::loadTexture(TextureLoaderRawTexture&& texture)
{
  gl_->glGenTextures(1, &texture_id_);
  gl_->glBindTexture(GL_TEXTURE_2D, texture_id_);
  gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texture.width, texture.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, &texture.image[0]);
  gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  gl_->glBindTexture(GL_TEXTURE_2D, 0);
}

void ResourceTexture::bind()
{
  gl_->glBindTexture(GL_TEXTURE_2D, texture_id_);
}
}
