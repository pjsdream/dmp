#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/object.h>

namespace dmp
{
void Environment::addObject(const std::shared_ptr<Object>& object)
{
  objects_.push_back(object);
  object_name_to_index_[object->getName()] = static_cast<int>(objects_.size()) - 1;
}

const std::vector<std::shared_ptr<Object>>& Environment::getObjects() const
{
  return objects_;
}

const std::shared_ptr<Object>& Environment::getObject(const std::string& object_name) const
{
  return objects_[object_name_to_index_.find(object_name)->second];
}
}
