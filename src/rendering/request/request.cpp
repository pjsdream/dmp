#include <dmp/rendering/request/request.h>
#include <dmp/json/json.h>

namespace dmp
{
std::shared_ptr<Json> Request::getJson() const noexcept
{
  return json_;
}

void Request::setJson(const std::shared_ptr<Json>& json)
{
  json_ = json;
}
}