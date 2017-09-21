#ifndef DMP_ROBOT_LINK_H
#define DMP_ROBOT_LINK_H

#include <string>
#include <Eigen/Dense>

namespace dmp
{
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

  struct Collision
  {
    std::string filename;
    Eigen::Affine3d transform;
  };

  RobotLink() = delete;
  explicit RobotLink(const std::string& name);

  const std::string& getName() const noexcept;

  void addVisual(const std::string& filename, const Eigen::Affine3d& transform);
  void addVisual(const std::string& filename,
                 const Eigen::Affine3d& transform,
                 bool has_color,
                 const Eigen::Vector4d& color);
  const std::vector<Visual> getVisuals() const noexcept;

  void addCollision(const std::string& filename, const Eigen::Affine3d& transform);

private:
  std::string name_;

  std::vector<Visual> visuals_;
  std::vector<Collision> collisions_;
};
}

#endif //DMP_ROBOT_LINK_H
