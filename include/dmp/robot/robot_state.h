#ifndef DMP_ROBOT_STATE_H
#define DMP_ROBOT_STATE_H

#include <vector>
#include <string>
#include <unordered_map>

#include <dmp/comm/message.h>

namespace dmp
{
class RobotState : public Message
{
public:
  RobotState() = delete;
  explicit RobotState(const std::vector<std::string>& joint_names);
  ~RobotState() override = default;

  double& position(int i);
  double& position(const std::string& joint_name);
  const double& position(int i) const;
  const double& position(const std::string& joint_name) const;

  double& velocity(int i);
  double& velocity(const std::string& joint_name);
  const double& velocity(int i) const;
  const double& velocity(const std::string& joint_name) const;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;

  std::unordered_map<std::string, int> joint_name_to_index_;
};
}

#endif //DMP_ROBOT_STATE_H
