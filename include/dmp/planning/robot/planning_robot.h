#ifndef DMP_PLANNING_ROBOT_H
#define DMP_PLANNING_ROBOT_H

#include <string>
#include <Eigen/Dense>

namespace dmp
{
class RobotModel;

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
  struct Link
  {
    int parent;
    std::string name;
    std::string filename;
    Eigen::Affine3d transform;
    bool has_color;
    Eigen::Vector4d color;
  };

  struct Joint
  {
    enum class Type
    {
      Fixed,
      Continuous,
      Revolute,
      Prismatic,
    };

    Type type;
    std::string name;
    Eigen::Affine3d origin;
    Eigen::Vector3d axis;
    double lower;
    double upper;
  };

  std::vector<Link> links_;
  std::vector<Joint> joints_;
};
}

#endif //DMP_PLANNING_ROBOT_H
