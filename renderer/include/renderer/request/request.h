#ifndef DMP_REQUEST_H
#define DMP_REQUEST_H

#include <core/comm/subscriber.h>

#include <string>
#include <memory>

namespace dmp
{
class Request
{
public:
  enum class Type : unsigned char
  {
    Clear = 0,
    Mesh,
    CustomMesh,
    CustomTexture,
    Shape,
    Frame,
    Light,
  };

  Request() = delete;

  explicit Request(Type type)
      : type_(type)
  {
  }

  virtual ~Request() = default;

  Request(const Request& rhs) = delete;
  Request& operator=(const Request& rhs) = delete;

  Request(Request&& rhs) = default;
  Request& operator=(Request&& rhs) = default;

  Type type() const
  { return type_; }

private:
  Type type_;
};
}

#endif //DMP_REQUEST_H
