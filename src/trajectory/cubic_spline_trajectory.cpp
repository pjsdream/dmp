#include <dmp/trajectory/cubic_spline_trajectory.h>
#include <dmp/trajectory/cubic_spline.h>

namespace dmp
{
CubicSplineTrajectory::CubicSplineTrajectory(const std::vector<std::string>& joint_names,
                                             double duration,
                                             int num_curves)
    : joint_names_(joint_names), num_curves_(num_curves)
{
  splines_.resize(joint_names.size(), CubicSpline(duration, num_curves));

  for (int i=0; i<joint_names_.size(); i++)
    joint_name_to_index_[joint_names_[i]] = i;
}

const std::vector<std::string>& CubicSplineTrajectory::getJointNames() const noexcept
{
  return joint_names_;
}

int CubicSplineTrajectory::numCurves() const noexcept
{
  return num_curves_;
}

CubicSpline& CubicSplineTrajectory::getSpline(int i)
{
  return splines_[i];
}

CubicSpline& CubicSplineTrajectory::getSpline(const std::string& joint_name)
{
  return splines_[joint_name_to_index_.find(joint_name)->second];
}
}
