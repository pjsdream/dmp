#ifndef DMP_REQUEST_CUSTOM_TEXTURE_H
#define DMP_REQUEST_CUSTOM_TEXTURE_H

#include <dmp/rendering/request/request.h>

#include <vector>

namespace dmp
{
class RequestCustomTexture : public Request
{
public:
  RequestCustomTexture();
  ~RequestCustomTexture() override = default;

  std::string name;
  unsigned int w;
  unsigned int h;
  std::vector<unsigned char> image;

private:
};
}

#endif //DMP_REQUEST_CUSTOM_TEXTURE_H
