#ifndef DMP_REQUEST_H
#define DMP_REQUEST_H

#include <string>
#include <memory>

#include <dmp/comm/message.h>

namespace dmp
{
class Request : public Message
{
public:
  Request() = default;
  virtual ~Request() override = default;

  Request(const Request& rhs) = delete;
  Request& operator=(const Request& rhs) = delete;

  Request(Request&& rhs) = default;
  Request& operator=(Request&& rhs) = default;

private:
};
}

#endif //DMP_REQUEST_H
