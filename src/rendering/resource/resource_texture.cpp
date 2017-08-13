#include <dmp/rendering/resource/resource_texture.h>

#include <lodepng/lodepng.h>

namespace dmp
{
ResourceTexture::ResourceTexture(const std::shared_ptr<GlFunctions>& gl)
    : Resource(gl)
{
}

ResourceTexture::~ResourceTexture()
{
  // TODO: delete gl resources
}

void ResourceTexture::loadTexture(const std::string& filename)
{
  std::vector<unsigned char> image;
  unsigned width, height;
  lodepng::decode(image, width, height, filename.c_str());

  gl_->glGenTextures(1, &texture_id_);
  gl_->glBindTexture(GL_TEXTURE_2D, texture_id_);
  gl_->glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, &image[0]);
  gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  gl_->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  gl_->glBindTexture(GL_TEXTURE_2D, 0);
}

void ResourceTexture::bind()
{
  gl_->glBindTexture(GL_TEXTURE_2D, texture_id_);
}
}
