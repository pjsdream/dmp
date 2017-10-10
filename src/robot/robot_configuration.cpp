#include <include/dmp/robot/robot_configuration.h>
#include <dmp/robot/planning_robot_model.h>
#include <dmp/robot/planning_robot_joint.h>
#include <dmp/robot/planning_robot_link.h>
#include <dmp/planning/motion/motion.h>

namespace dmp
{
RobotConfiguration::RobotConfiguration(const std::shared_ptr<PlanningRobotModel>& robot_model)
    : robot_model_(robot_model)
{
  transforms_.resize(robot_model_->numLinks());
}

void RobotConfiguration::setPositions(const Eigen::VectorXd& positions) noexcept
{
  positions_ = positions;
}

const Eigen::VectorXd& RobotConfiguration::getPositions() const noexcept
{
  return positions_;
}

void RobotConfiguration::setVelocities(const Eigen::VectorXd& velocities) noexcept
{
  velocities_ = velocities;
}

const Eigen::VectorXd& RobotConfiguration::getVelocities() const noexcept
{
  return velocities_;
}

void RobotConfiguration::forwardKinematics()
{
  // TODO
  /*
  const auto num_links = robot_model_->numLinks();
  for (int i = 0; i < num_links; i++)
  {
    const auto& link = robot_model_->getLink(i);

    Eigen::Affine3d transform;
    if (i == 0)
      transforms_[i] = Eigen::Affine3d::Identity();
    else
    {
      const auto& joint = robot_model_->getJoint(i);
      const auto parent = robot_model_->getParent(i);

      transforms_[i].matrix().noalias() = transforms_[parent].matrix()
          * joint.getJointTransform(positions_(i)).matrix();
    }
  }
   */
}

const Eigen::Affine3d& RobotConfiguration::getTransform(int link_index) const
{
  return transforms_[link_index];
}
}
