#ifndef DMP_ROBOT_MODEL_LOADER_H
#define DMP_ROBOT_MODEL_LOADER_H

#include <string>
#include <memory>

namespace dmp
{
class RobotModel;
class RobotModelLoader
{
public:
  RobotModelLoader() = default;
  ~RobotModelLoader() = default;

  std::shared_ptr<RobotModel> load(const std::string& filename);

private:
};
}

#endif //DMP_ROBOT_MODEL_LOADER_H
