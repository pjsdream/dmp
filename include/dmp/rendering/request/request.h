#ifndef DMP_REQUEST_H
#define DMP_REQUEST_H

#include <string>
#include <memory>

#include <dmp/json/json.h>

namespace dmp
{
class Request
{
public:
  Request() = default;
  explicit Request(const Json& json);
  explicit Request(Json&& json);
  ~Request() = default;

  Request(Request&& rhs) = default;
  Request& operator=(Request&& rhs) = default;

  Json getJson() const noexcept;
  void setJson(const Json& json);
  void setJson(Json&& json);

private:
  Json json_;
};
}

#endif //DMP_REQUEST_H
