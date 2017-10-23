#ifndef DMP_REQUEST_H
#define DMP_REQUEST_H

#include <string>
#include <memory>

namespace dmp
{
class Request
{
public:
  enum class RequestType : unsigned char
  {
    Clear = 0,
    Mesh,
    CustomMesh,
    CustomTexture,
    Shape,
    Frame,
    Light,
  };

  Request() = default;
  virtual ~Request() = default;

  Request(const Request& rhs) = delete;
  Request& operator=(const Request& rhs) = delete;

  Request(Request&& rhs) = default;
  Request& operator=(Request&& rhs) = default;

private:
};
}

#endif //DMP_REQUEST_H
