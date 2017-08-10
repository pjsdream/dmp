#include <dmp/robot/robot_model_loader.h>
#include <dmp/robot/robot_model.h>

namespace dmp
{
std::shared_ptr<RobotModel> RobotModelLoader::load(const std::string& filename)
{
  // TODO
  return std::make_unique<RobotModel>();
}
}