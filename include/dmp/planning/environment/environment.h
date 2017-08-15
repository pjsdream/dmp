#ifndef DMP_ENVIRONMENT_H
#define DMP_ENVIRONMENT_H

#include <vector>
#include <memory>

namespace dmp
{
class Object;
class Environment
{
public:
private:
  std::vector<std::shared_ptr<Object>> objects_;
};
}

#endif //DMP_ENVIRONMENT_H
