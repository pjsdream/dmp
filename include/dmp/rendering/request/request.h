#ifndef DMP_REQUEST_H
#define DMP_REQUEST_H

#include <string>
#include <memory>

namespace dmp
{
class Json;
class Request
{
public:
  Request() = default;
  ~Request() = default;

  Request(Request&& rhs) = default;
  Request& operator=(Request&& rhs) = default;

  std::shared_ptr<Json> getJson() const noexcept;
  void setJson(const std::shared_ptr<Json>& json);

private:
  std::shared_ptr<Json> json_;
};
}

#endif //DMP_REQUEST_H
