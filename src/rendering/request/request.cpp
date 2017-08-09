#include <dmp/rendering/request/request.h>

namespace dmp
{
Request::Request()
    : action_(Action::Nothing)
{
}

void Request::setAction(Action action)
{
  action_ = action;
}

void Request::setObjectName(const std::string& name)
{
  object_name_ = name;
}
}