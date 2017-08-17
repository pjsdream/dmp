#include <dmp/planning/environment/environment_loader.h>
#include <dmp/planning/environment/environment.h>
#include <dmp/planning/environment/interactable_object.h>
#include <dmp/planning/environment/obstacle.h>
#include <dmp/json/json.h>
#include <dmp/json/json_loader.h>

namespace dmp
{
std::shared_ptr<Environment> EnvironmentLoader::loadEnvironment(const std::string& filename)
{
  JsonLoader json_loader;
  Json json{json_loader.loadJson(filename)};

  printf("%s\n", json.toPrettyString().c_str());

  // TODO
  return nullptr;
}
}
