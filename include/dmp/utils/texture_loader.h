#ifndef DMP_TEXTURE_LOADER_H
#define DMP_TEXTURE_LOADER_H

#include <future>
#include <vector>
#include <string>

namespace dmp
{
struct TextureLoaderRawTexture
{
  unsigned int width;
  unsigned int height;
  std::vector<unsigned char> image;
};

class TextureLoader
{
public:
  static std::future<TextureLoaderRawTexture> asyncLoadTexture(const std::string& filename);

private:
};
}

#endif //DMP_TEXTURE_LOADER_H
