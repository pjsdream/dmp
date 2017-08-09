#ifndef DMP_REQUEST_H
#define DMP_REQUEST_H

#include <string>

namespace dmp
{
class Request
{
public:
  enum class Action
  {
    Nothing,
    Add,
  };

  Request();
  ~Request() = default;

  void setAction(Action action);
  void setObjectName(const std::string& name);

private:
  Action action_;
  std::string object_name_;
};
}

#endif //DMP_REQUEST_H
