#ifndef DMP_REQUEST_CUSTOM_TEXTURE_H
#define DMP_REQUEST_CUSTOM_TEXTURE_H

#include "request.h"

#include <vector>

namespace dmp
{
class RequestCustomTexture : public Request
{
public:
  RequestCustomTexture() : Request(Request::Type::CustomTexture)
  {
  }

  ~RequestCustomTexture() override = default;

  std::string name;
  unsigned int w;
  unsigned int h;
  std::vector<unsigned char> image;

  template<typename Archive>
  Archive& serialize(Archive& ar)
  {
    return ar & name & w & h & image;
  }

private:
};
}

#endif //DMP_REQUEST_CUSTOM_TEXTURE_H
