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
  void addObject(const std::shared_ptr<Object>& object);

  const std::vector<std::shared_ptr<Object>>& getObjects() const;

private:
  std::vector<std::shared_ptr<Object>> objects_;
};
}

#endif //DMP_ENVIRONMENT_H
