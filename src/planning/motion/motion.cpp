#include <dmp/planning/motion/motion.h>

namespace dmp
{
void Motion::setNavigationJoints(const std::vector<std::string>& navigation_joints)
{
  navigation_joints_ = navigation_joints;
}

void Motion::setBodyJoints(const std::vector<std::string>& body_joints)
{
  body_joints_ = body_joints;
}

void Motion::setGripper(const std::vector<std::string>& gripper_joints,
                        const std::string& link,
                        const Eigen::Vector3d& xyz,
                        double width)
{
  gripper_joints_ = gripper_joints;
  gripper_link_ = link;
  gripper_link_xyz_ = xyz;
  gripper_width_ = width;
}
}
