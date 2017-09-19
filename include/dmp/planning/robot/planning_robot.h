#ifndef DMP_PLANNING_ROBOT_H
#define DMP_PLANNING_ROBOT_H

#include <string>
#include <memory>
#include <Eigen/Dense>

namespace dmp
{
class RobotModel;
class RobotLink;
class RobotJoint;

class PlanningRobot
{
public:
  PlanningRobot() = delete;
  explicit PlanningRobot(const RobotModel& robot_model);
  ~PlanningRobot() = default;

  PlanningRobot(const PlanningRobot& rhs) = default;
  PlanningRobot& operator=(const PlanningRobot& rhs) = default;

  PlanningRobot(PlanningRobot&& rhs) = default;
  PlanningRobot& operator=(PlanningRobot&& rhs) = default;

private:
  void buildFromRobotModel(const std::shared_ptr<RobotLink>& model_link, int parent_link_id = -1);

  struct Link
  {
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

    int parent;
    std::string name;

    std::vector<Visual> visuals;
    std::vector<Collision> collisions;
  };

  struct Joint
  {
    enum class Type
    {
      Fixed,
      Continuous,
      Revolute,
      Prismatic,
      Undefined,
    };

    Type type;
    std::string name;
    Eigen::Affine3d origin;
    Eigen::Vector3d axis;
    double lower;
    double upper;
  };

  static Joint::Type getJointType(std::string type);

  std::vector<Link> links_;
  std::vector<Joint> joints_;
};
}

#endif //DMP_PLANNING_ROBOT_H
