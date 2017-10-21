#ifndef DMP_ENVIRONMENT_H
#define DMP_ENVIRONMENT_H

#include <vector>
#include <memory>
#include <unordered_map>

namespace dmp
{
class Object;
class Environment
{
public:
  void addObject(const std::shared_ptr<Object>& object);

  const std::vector<std::shared_ptr<Object>>& getObjects() const;
  const std::shared_ptr<Object>& getObject(const std::string& object_name) const;

private:
  std::vector<std::shared_ptr<Object>> objects_;
  std::unordered_map<std::string, int> object_name_to_index_;
};
}

#endif //DMP_ENVIRONMENT_H
