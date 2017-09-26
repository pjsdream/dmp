#include <dmp/robot/robot_state.h>

namespace dmp
{
RobotState::RobotState(const std::vector<std::string>& joint_names) noexcept
    : joint_names_(joint_names)
{
  for (int i=0; i<joint_names.size(); i++)
    joint_name_to_index_[joint_names[i]] = i;
}

double& RobotState::position(int i)
{
  return positions_[i];
}

double& RobotState::position(const std::string& joint_name)
{
  return positions_[joint_name_to_index_.find(joint_name)->second];
}

const double& RobotState::position(int i) const
{
  return positions_[i];
}

const double& RobotState::position(const std::string& joint_name) const
{
  return positions_[joint_name_to_index_.find(joint_name)->second];
}

double& RobotState::velocity(int i)
{
  return velocities_[i];
}

double& RobotState::velocity(const std::string& joint_name)
{
  return velocities_[joint_name_to_index_.find(joint_name)->second];
}

const double& RobotState::velocity(int i) const
{
  return velocities_[i];
}

const double& RobotState::velocity(const std::string& joint_name) const
{
  return velocities_[joint_name_to_index_.find(joint_name)->second];
}
}