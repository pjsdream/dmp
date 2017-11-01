#ifndef DMP_MOTION_H
#define DMP_MOTION_H

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace dmp
{
class Motion
{
public:
  void setNavigationJoints(const std::vector<std::string>& navigation_joints);

  void setBodyJoints(const std::vector<std::string>& body_joints);
  const std::vector<std::string>& getBodyJoints();

  void setGripper(const std::vector<std::string>& gripper_joints,
                  const std::string& link,
                  const Eigen::Vector3d& xyz,
                  double width);
  const std::string& getGripperLink() const noexcept;
  const Eigen::Vector3d getGripperXyz() const noexcept;

private:
  std::vector<std::string> navigation_joints_;
  std::vector<std::string> body_joints_;
  std::vector<std::string> gripper_joints_;
  std::string gripper_link_;
  Eigen::Vector3d gripper_link_xyz_;
  double gripper_width_;
};
}

#endif //DMP_MOTION_H
