#include <dmp/rendering/request/request.h>
#include <dmp/json/json.h>

namespace dmp
{
Request::Request(const Json& json)
{
  setJson(json);
}

Request::Request(Json&& json)
{
  setJson(std::move(json));
}

Json Request::getJson() const noexcept
{
  return json_;
}

void Request::setJson(const Json& json)
{
  json_ = json;
}

void Request::setJson(Json&& json)
{
  json_ = std::move(json);
}
}
