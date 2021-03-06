#ifndef DMP_TREE_ROBOT_LINK_H
#define DMP_TREE_ROBOT_LINK_H

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace dmp
{
class TreeRobotJoint;
class TreeRobotLink
{
public:
  struct Visual
  {
    std::string filename;
    Eigen::Affine3d transform;
    bool has_color;
    Eigen::Vector4d color;
  };

  struct Collision
  {
    std::string filename;
    Eigen::Affine3d transform;
  };

  void setName(const std::string& name);
  const std::string& getName() const;

  void setParentJoint(const std::shared_ptr<TreeRobotJoint>& parent);
  void addChildJoint(const std::shared_ptr<TreeRobotJoint>& chlid);
  const std::vector<std::shared_ptr<TreeRobotJoint>>& getChildJoints() const;

  void addVisualMesh(const std::string& filename, const Eigen::Affine3d& transform);
  void addVisualMesh(const std::string& filename, const Eigen::Affine3d& transform, const Eigen::Vector4d& color);
  const std::vector<Visual>& getVisuals() const noexcept;

  void addCollisionMesh(const std::string& filename, const Eigen::Affine3d& transform) noexcept;
  const std::vector<Collision>& getCollisions() const noexcept;

private:
  std::string name_;

  std::shared_ptr<TreeRobotJoint> parent_;
  std::vector<std::shared_ptr<TreeRobotJoint>> children_;

  std::vector<Visual> visuals_;
  std::vector<Collision> collisions_;
};
}

#endif //DMP_TREE_ROBOT_LINK_H
