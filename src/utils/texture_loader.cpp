#include <dmp/utils/texture_loader.h>
#include <lodepng/lodepng.h>

namespace dmp
{
std::future<TextureLoaderRawTexture> TextureLoader::asyncLoadTexture(const std::string& filename)
{
  return std::async(std::launch::async, [filename](){
    TextureLoaderRawTexture texture;
    lodepng::decode(texture.image, texture.width, texture.height, filename.c_str());
    return texture;
  });
}
}