#ifndef DMP_ROBOT_MODEL_H
#define DMP_ROBOT_MODEL_H

#include <memory>
#include <unordered_map>
#include <Eigen/Dense>
#include <dmp/utils/vector_eigen.h>
#include <Eigen/StdVector>

namespace dmp
{
class TreeRobotModel;
class TreeRobotLink;
class RobotLink;
class RobotJoint;

class RobotModel
{
public:
  RobotModel() = delete;
  explicit RobotModel(const TreeRobotModel& tree_robot_model) noexcept;
  ~RobotModel();

  RobotModel(const RobotModel& rhs) = default;
  RobotModel& operator=(const RobotModel& rhs) = default;

  RobotModel(RobotModel&& rhs) = default;
  RobotModel& operator=(RobotModel&& rhs) = default;

  int numLinks() const noexcept;
  int numJoints() const noexcept;
  std::vector<std::string> getJointNames() const;

  const RobotLink& getLink(int index) const noexcept;
  const RobotJoint& getJoint(int index) const noexcept; // index corresponds to index of link
  const RobotJoint& getJoint(const std::string& joint_name) const noexcept;
  int getParent(int index) const noexcept;

  int getLinkIndex(const std::string& link_name) const;

  VectorEigen<Eigen::Affine3d> forwardKinematics(const std::vector<double>& joint_values) const;

private:
  void buildFromRobotModel(const std::shared_ptr<TreeRobotLink>& model_link, int parent_link_id = -1);

  std::vector<int> parents_;
  std::vector<RobotLink> links_;
  std::vector<RobotJoint> joints_;

  std::unordered_map<std::string, int> joint_name_to_index_;
  std::unordered_map<std::string, int> link_name_to_index_;
};
}

#endif //DMP_ROBOT_MODEL_H
