#ifndef DMP_MOTION_LOADER_H
#define DMP_MOTION_LOADER_H

#include <string>
#include <vector>
#include <memory>

#include <Eigen/Dense>

namespace dmp
{
class Motion;
class MotionLoader
{
public:
  std::shared_ptr<Motion> load(const std::string& filename);

private:
  std::vector<std::string> navigation_joints_;
  std::vector<std::string> body_joints_;
  std::vector<std::string> gripper_joints_;

  std::string gripping_position_link_;
  Eigen::Vector3d gripping_position_offset_;
  double gripping_position_width_;
};
}

#endif //DMP_MOTION_LOADER_H
