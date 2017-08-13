#ifndef DMP_ROBOT_LINK_H
#define DMP_ROBOT_LINK_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace dmp
{
class RobotJoint;
class RobotLink
{
public:
  struct Visual
  {
    std::string filename;
    Eigen::Affine3d transform;
    bool has_color;
    Eigen::Vector4d color;
  };

  void setName(const std::string& name);

  void setParentJoint(const std::shared_ptr<RobotJoint>& parent);
  void addChildJoint(const std::shared_ptr<RobotJoint>& chlid);

  void addVisualMesh(const std::string& filename, const Eigen::Affine3d& transform);
  void addVisualMesh(const std::string& filename, const Eigen::Affine3d& transform, const Eigen::Vector4d& color);

  const std::string& getName();
  const std::vector<Visual>& getVisuals();
  const std::vector<std::shared_ptr<RobotJoint>>& getChildJoints();

private:
  std::string name_;

  std::shared_ptr<RobotJoint> parent_;
  std::vector<std::shared_ptr<RobotJoint>> children_;

  std::vector<Visual> visuals_;
};
}

#endif //DMP_ROBOT_LINK_H
