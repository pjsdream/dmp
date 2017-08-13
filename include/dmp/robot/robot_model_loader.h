#ifndef DMP_ROBOT_MODEL_LOADER_H
#define DMP_ROBOT_MODEL_LOADER_H

#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Dense>

namespace dmp
{
class RobotModel;
class RobotLink;
class RobotJoint;
class RobotModelLoader
{
public:
  RobotModelLoader() = default;
  ~RobotModelLoader() = default;

  void setSubstitutePackageDirectory(const std::string& directory);
  void load(const std::string& filename);

  void setAllJointsActive();
  void setActiveJoints(const std::vector<std::string>& active_joints);
  void setJointValues(const std::unordered_map<std::string, double>& joint_values);

  std::shared_ptr<RobotModel> getRobotModel();

private:
  std::string package_directory_;

  struct Link
  {
    struct Inertial
    {
      double origin[6];
      double mass;
      double inertia[6];
    };

    struct Visual
    {
      double origin[6];
      std::string geometry_filename;
      bool has_material;
      double material_color[4];
    };

    struct Collision
    {
      double origin[6];
      std::string geometry_filename;
    };

    std::string name;
    Inertial inertial;
    Visual visual;
    Collision collision;
  };

  struct Joint
  {
    struct Limit
    {
      double effort;
      double lower;
      double upper;
      double velocity;
    };

    std::string name;
    std::string type;
    double origin[6];
    std::string parent;
    std::string child;
    double axis[3];
    Limit limit;
  };

  std::string robot_name_;
  std::unordered_map<std::string, Link> links_;
  std::unordered_map<std::string, Joint> joints_;
  std::unordered_map<std::string, int> num_parents_;

  std::unordered_map<std::string, std::unordered_set<std::string>> children_joints_;

  std::string root_name_;

  std::unordered_set<std::string> active_joints_;
  std::unordered_map<std::string, double> joint_values_;
  void traverse(const std::shared_ptr<RobotLink>& node, const std::string& link_name, const Eigen::Affine3d& transform);

  std::string substitutePackageDirectory(const std::string& filename);
  Eigen::Affine3d originToTransform(const double origin[6]);
  std::shared_ptr<RobotJoint> createRobotJointFromRaw(const Joint& raw_joint);
};
}

#endif //DMP_ROBOT_MODEL_LOADER_H
