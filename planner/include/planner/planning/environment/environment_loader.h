#ifndef DMP_ENVIRONMENT_LOADER_H
#define DMP_ENVIRONMENT_LOADER_H

#include <memory>

namespace dmp
{
class Environment;
class EnvironmentLoader
{
public:
  std::shared_ptr<Environment> loadEnvironment(const std::string& filename);

private:
};
}

#endif //DMP_ENVIRONMENT_LOADER_H
