#ifndef DMP_CUBIC_SPLINE_TRAJECTORY_H
#define DMP_CUBIC_SPLINE_TRAJECTORY_H

#include <string>
#include <vector>
#include <unordered_map>

namespace dmp
{
class CubicSpline;

class CubicSplineTrajectory
{
public:
  CubicSplineTrajectory() = delete;
  CubicSplineTrajectory(const std::vector<std::string>& joint_names, double duration, int num_curves);

  const std::vector<std::string>& getJointNames() const noexcept;

  CubicSpline& getSpline(const std::string& joint_name);

private:
  double duration_;
  std::vector<std::string> joint_names_;

  std::vector<CubicSpline> splines_;

  std::unordered_map<std::string, int> joint_name_to_index_;
};
}

#endif //DMP_CUBIC_SPLINE_TRAJECTORY_H
