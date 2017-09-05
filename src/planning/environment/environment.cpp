#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/object.h>

namespace dmp
{
void Environment::addObject(const std::shared_ptr<Object>& object)
{
  objects_.push_back(object);
}

const std::vector<std::shared_ptr<Object>>& Environment::getObjects()
{
  return objects_;
}
}
